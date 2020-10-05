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
      const uint32_t period = 10000000;
      uint32_t pulseWidth1 = 1500000;
      uint32_t pulseWidth2 = 1500000;
      uint8_t enable = 1;
      char setValue[9];
      FILE *myOutputHandle;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        bind<IMC::SetThrusterActuation>(this);
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm0/period", "ab")) == NULL)
        {
          throw std::runtime_error("Unable to set PWM0 period.");
          return;
        }
	sprintf(setValue, "%u", period);
        fwrite(&setValue, sizeof(char), 7, myOutputHandle);
        fwrite(&setValue, sizeof(char), 8, myOutputHandle);
        fclose(myOutputHandle);
        if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm1/period", "ab")) == NULL)
        {
          throw std::runtime_error("Unable to set PWM1 period.");
          return;
        }
        fwrite(&setValue, sizeof(char), 8, myOutputHandle);
        fclose(myOutputHandle);
        if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", "ab")) == NULL)
        {
          throw std::runtime_error("Unable to set PWM0 duty-cycle.");
          return;
        }
	sprintf(setValue, "%u", pulseWidth1);
        fwrite(&setValue, sizeof(char), 7, myOutputHandle);
        fclose(myOutputHandle);
        if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm1/duty_cycle", "ab")) == NULL)
        {
          throw std::runtime_error("Unable to set PWM1 duty-cycle.");
          return;
        }
        sprintf(setValue, "%u", pulseWidth2);
        fwrite(&setValue, sizeof(char), 7, myOutputHandle);
        fclose(myOutputHandle);
        if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm0/enable", "ab")) == NULL)
        {
          throw std::runtime_error("Unable to set PWM0 enable.");
          return;
        }
        fwrite(&enable, sizeof(uint8_t), 1, myOutputHandle);
        fclose(myOutputHandle);
        if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm1/enable", "ab")) == NULL)
        {
          throw std::runtime_error("Unable to set PWM1 enable.");
          return;
        }
        fwrite(&enable, sizeof(uint8_t), 1, myOutputHandle);
        fclose(myOutputHandle);
      }

      void
      consume(const IMC::SetThrusterActuation* msg)
      {
        if(msg->id == 0)
        {
          pulseWidth1 = (1100 + 400 * (msg->value + 1)) * 1000;
          if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", "ab")) == NULL)
          {
            throw std::runtime_error("Unable to set PWM0 duty-cycle.");
            return;
          }
          sprintf(setValue, "%u", pulseWidth1);
          fwrite(&setValue, sizeof(char), 7, myOutputHandle);
          fclose(myOutputHandle);
        }
        else if(msg->id == 1)
        {
          pulseWidth2 = (1100 + 400 * (msg->value + 1)) * 1000;
          if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm1/duty_cycle", "ab")) == NULL)
          {
            throw std::runtime_error("Unable to set PWM1 duty-cycle.");
            return;
          }
          sprintf(setValue, "%u", pulseWidth2);
          fwrite(&setValue, sizeof(char), 7, myOutputHandle);
          fclose(myOutputHandle);
        }
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        enable = 0;
        if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm0/enable", "ab")) == NULL)
        {
          throw std::runtime_error("Unable to set PWM0 enable.");
          return;
        }
        fwrite(&enable, sizeof(uint8_t), 1, myOutputHandle);
        fclose(myOutputHandle);
        if ((myOutputHandle = fopen("/sys/class/pwm/pwmchip0/pwm1/enable", "ab")) == NULL)
        {
          throw std::runtime_error("Unable to set PWM1 enable.");
          return;
        }
        fwrite(&enable, sizeof(uint8_t), 1, myOutputHandle);
        fclose(myOutputHandle);
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          consumeMessages();
        }
      }
    };
  }
}

DUNE_TASK
