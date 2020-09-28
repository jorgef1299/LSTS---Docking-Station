//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Jorge Ferreira                                                    *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

//! Flags for status register #1.
#define STAT_DRDY 0b00000001  // Data Ready.
#define STAT_OVL 0b00000010   // Overflow flag.
#define STAT_DOR 0b00000100   // Data skipped for reading.

namespace Sensors
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Jorge Ferreira
  namespace QMC5883L
  {
    using DUNE_NAMESPACES;
    
    //! Task arguments.
    struct Arguments
    {
      //! I2C device.
      std::string i2c_dev;
      //! Offset bias correction value.
      std::vector<int16_t> offset_bias;
      //! Scale correction factors.
      std::vector<float> scale_correction;
      
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! I2C handle.
      I2C* m_i2c;
      //! Device I2C address.
      static const uint8_t dev_addr = 0x0d;
      //! Data registers of the magnetic sensor
      const uint8_t xRegisterLSB = 0x00;
      const uint8_t xRegisterMSB = 0x01;
      const uint8_t yRegisterLSB = 0x02;
      const uint8_t yRegisterMSB = 0x03;
      const uint8_t zRegisterLSB = 0x04;
      const uint8_t zRegisterMSB = 0x05;
      const uint8_t tRegisterLSB = 0x07;
      const uint8_t statusRegister = 0x06;
      //! Magnetic field.
      IMC::MagneticField m_magn;
      //! Task arguments.
      Arguments m_args;
      //! Device protocol handler.
      Hardware::LUCL::Protocol m_proto;
      
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        // Define configuration parameters.
        param("I2C - Device", m_args.i2c_dev)
        .defaultValue("")
        .description("I2C Device");
        
        param("Magnetometer Offset Bias", m_args.offset_bias)
        .defaultValue("")
        .size(3)
        .description("Offset bias correction value");
        
        param("Magnetometer Scale Correction", m_args.scale_correction)
        .defaultValue("")
        .size(3)
        .description("Scale correction value");
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        m_i2c = new I2C(m_args.i2c_dev);
        m_i2c->connect(dev_addr);
        
        // Read chip id.
        uint8_t chipID = 0;
        uint8_t reg = 0x0d;
        m_i2c->write(&reg, 1);
        m_i2c->read(&chipID, 1);
        if(chipID != 0xFF)
          throw std::runtime_error("Chip ID is wrong.");
        
        // Set the device in continuous read mode.
        reg = 0x0a;
        uint8_t data2[2];
        data2[0] = reg;
        data2[1] = 0b10000000;
        m_i2c->write(data2, 2);
        
        data2[1] = 0b00000001;
        m_i2c->write(data2, 2);
        
        data2[0] = 0x0b;
        data2[1] = 0x01;
        m_i2c->write(data2, 2);
        
        data2[0] = 0x09;
        data2[1] = 0b00000001;
        m_i2c->write(data2, 2);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }
      
      //! Read one byte
      uint8_t
      readByte(const uint8_t* register_addr)
      {
        uint8_t value;
        m_i2c->write(register_addr, 1);
        m_i2c->read(&value, 1);
        
        return value;
      }
      
      //! Read a two bytes value, stored as LSB and MSB, in 2's complement.
      int16_t
      readWord(const uint8_t* register_addr)
      {
        uint8_t lowByte = readByte(register_addr);
        uint8_t highByte = readByte(register_addr + 1);
        uint16_t word = (highByte << 8) + lowByte;
        
        if(word >= 32768)
          return (word - 65536);
        
        return word;
      } 
      
      //! Read raw data from the magnetometer
      void
      readInput(void)
      {
        uint8_t status;
        uint8_t i = 0;
        int16_t mag_x = 0;
        int16_t mag_y = 0;
        int16_t mag_z = 0;
        float mag_x2;
        float mag_y2;
        float mag_z2;
        double imc_tstamp;
        
        while(i < 20)
        {
          status = readByte(&statusRegister);
          if(status & STAT_OVL)
            throw std::runtime_error(String::str("Magnetic sensor overflow. Please switch to RNG_8G output range."));
          if(status & STAT_DOR)
          {
            mag_x = readWord(&xRegisterLSB);
            mag_y = readWord(&yRegisterLSB);
            mag_z = readWord(&zRegisterLSB);
            continue;
          }
          if(status & STAT_DRDY)
          {
            mag_x = readWord(&xRegisterLSB);
            mag_y = readWord(&yRegisterLSB);
            mag_z = readWord(&zRegisterLSB);
            break;
          }
          Time::Delay::waitMsec(10);
          i++;
        }
        
        // Remove offset bias and rescale.
        mag_x2 = (float) ((mag_x - m_args.offset_bias[0]) * m_args.scale_correction[0]);
        mag_y2 = (float) ((mag_y - m_args.offset_bias[1]) * m_args.scale_correction[1]);
        mag_z2 = (float) ((mag_z - m_args.offset_bias[2]) * m_args.scale_correction[2]);
        
        imc_tstamp = Clock::getSinceEpoch();
        m_magn.setTimeStamp(imc_tstamp);
        m_magn.x = (float) mag_x2 / 1000;
        m_magn.y = (float) mag_y2 / 1000;
        m_magn.z = (float) mag_z2 / 1000;
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          readInput();
          dispatch(m_magn);
          inf("%.6f\t%.6f\t%.6f", m_magn.x, m_magn.y, m_magn.z);
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
