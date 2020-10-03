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

namespace Actuators
{
  namespace BR_T200
  {
    using DUNE_NAMESPACES;
    
    struct Task: public DUNE::Tasks::Task
    {
      //! Name of pin to use.
      char GPIOString[4];
      //! Value to put in pinout.
      char GPIOValue1[64];
      char GPIOValue2[64];
      //! Mode in/out of pinout.
      char setValue[4];
      char GPIODirection[64];
      //! Handle of pwm pinout.
      FILE* myOutputHandle;
      //! Desired pulse width.
      uint16_t desiredPW1 = 1500;
      uint16_t desiredPW2 = 1500;
      
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        bind<IMC::SetThrusterActuation>(this);
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        initGPIO(16, 1);
        initGPIO(20, 2);
      }

      bool
      initGPIO(int gpioPin, uint8_t thrusterNumber)
      {
        sprintf(GPIOString, "%d", gpioPin);
        sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", gpioPin);
        if(thrusterNumber == 1)
          sprintf(GPIOValue1, "/sys/class/gpio/gpio%d/value", gpioPin);
        else if(thrusterNumber == 2)
          sprintf(GPIOValue2, "/sys/class/gpio/gpio%d/value", gpioPin);
        // Export the pin
        if ((myOutputHandle = fopen("/sys/class/gpio/export", "ab")) == NULL)
        {
          throw std::runtime_error("Unable to export GPIO pin");
          return false;
        }
        strcpy(setValue, GPIOString);
        fwrite(&setValue, sizeof(char), 2, myOutputHandle);
        fclose(myOutputHandle);
        // Set direction of the pin to an output
        if ((myOutputHandle = fopen(GPIODirection, "rb+")) == NULL)
        {
          throw std::runtime_error("Unable to open direction handle");
          return false;
        }
        strcpy(setValue,"out");
        fwrite(&setValue, sizeof(char), 3, myOutputHandle);
        fclose(myOutputHandle);

        return true;
      }
      
      void
      consume(const IMC::SetThrusterActuation* msg)
      {
        if(msg->id == 1)
          desiredPW1 = 1100 + 400 * (msg->value + 1);
        else if(msg->id == 2)
          desiredPW2 = 1100 + 400 * (msg->value + 1);
      }
      
      void
      setPW(uint8_t thrusterNumber, uint16_t pwValue, char* gpioValue)
      {
        // Set output to high.
        if ((myOutputHandle = fopen(gpioValue, "rb+")) == NULL)
          throw std::runtime_error("ERROR PinOut!");
        strcpy(setValue, "1"); // Set value high
        fwrite(&setValue, sizeof(char), 1, myOutputHandle);
        fclose(myOutputHandle);
        Delay::waitUsec(pwValue);
        inf("UM");
        
        // Set output to low.
        if ((myOutputHandle = fopen(gpioValue, "rb+")) == NULL)
          throw std::runtime_error("ERROR PinOut!");
        strcpy(setValue, "0"); // Set value low
        fwrite(&setValue, sizeof(char), 1, myOutputHandle);
        fclose(myOutputHandle);;
        Delay::waitUsec(8000 - pwValue);
        inf("Zero");
      }

      //! Main loop.
      void
      onMain(void)
      {
        setPW(1, 1500, GPIOValue1);
        setPW(2, 1500, GPIOValue1);
        Delay::wait(5);
        while (!stopping())
        {
          consumeMessages();
          setPW(1, desiredPW1, GPIOValue1);
          setPW(2, desiredPW2, GPIOValue2);
        }
      }
    };
  }
}

DUNE_TASK
