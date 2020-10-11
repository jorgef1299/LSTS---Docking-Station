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
// Author: Jorge Ferreira & Filipe Oliveira                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace LiDAR
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Jorge Ferreira & Filipe Oliveira
  using DUNE_NAMESPACES;

  struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
    };

  struct Task: public DUNE::Tasks::Task
  {
    const int HEADER = 0x59;
    int check;
    IMC::Distance distance;
    //! Serial port handle.
    SerialPort* m_uart;
    //! Task arguments
    Arguments m_args;
    //! Constructor.
    //! @param[in] name task name.
    //! @param[in] ctx context.
    Task(const std::string& name, Tasks::Context& ctx):
      DUNE::Tasks::Task(name, ctx)
    {
      param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

      param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");
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
      m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
      m_uart->setCanonicalInput(true);
    }

    //! Read LiDAR
    void
    onLiDARread(void)
    {
      uint8_t m_buffer;
      int i;
      int uart[9];
      if(m_uart->read(&m_buffer, 1)==HEADER)
      {
        uart[0]=HEADER;
        if(m_uart->read(&m_buffer, 1)==HEADER)
        {
          uart[1]=HEADER;
          for(i=2;i<9;i++)
          {
            uart[i]=m_uart->read(&m_buffer, 1);
          }
          check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
          if(uart[8]==(check&0xff))
          {
            distance.value=uart[2]+uart[3]*256;// calculate distance value
            dispatch(distance);// output Lidar tests distance value
            Time::Delay::waitMsec(100);
          }
        }
      }
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

    //! Main loop.
    void
    onMain(void)
    {
      while (!stopping())
      {
        waitForMessages(1.0);
        onLiDARread();
      }
    }
  };
}

DUNE_TASK
