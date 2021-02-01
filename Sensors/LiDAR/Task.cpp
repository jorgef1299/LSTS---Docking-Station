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
// Author: Jorge Ferreira                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local header
#include "Reader.hpp"

namespace Sensors {
namespace LiDAR {
// Device driver for TFmini plus LiDAR
using DUNE_NAMESPACES;

struct Arguments {
  //! Serial port device.
  std::string uart_dev;
  //! Serial port baud rate.
  unsigned uart_baud;
};

struct Task : public DUNE::Tasks::Task {
  //! Serial port handle.
  IO::Handle *m_handle;
  //! Reader thread.
  Reader *m_reader;
  //! Distance message.
  IMC::Distance distance;
  //! Task arguments
  Arguments m_args;

  const int HEADER = 0x59;
  int check;

  Task(const std::string &name, Tasks::Context &ctx)
      : Tasks::Task(name, ctx), m_handle(NULL), m_reader(NULL) {
    param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

    param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");

    bind<IMC::IoEvent>(this);
  }

  //! Update internal state with new parameter values.
  void onUpdateParameters(void) {}

  //! Reserve entity identifiers.
  void onEntityReservation(void) {}

  //! Resolve entity names.
  void onEntityResolution(void) {}

  //! Acquire resources.
  void onResourceAcquisition(void) {
    m_handle = new SerialPort(m_args.uart_dev, m_args.uart_baud);
    m_reader = new Reader(this, m_handle);
    m_reader->start();
  }

  //! Initialize resources.
  void onResourceInitialization(void) {}

  //! Release resources.
  void onResourceRelease(void) {
    if (m_reader != NULL) {
      m_reader->stopAndJoin();
      delete m_reader;
      m_reader = NULL;
    }

    Memory::clear(m_handle);
  }

  void consume(const IMC::IoEvent *msg) {
    if (msg->getDestination() != getSystemId())
      return;

    if (msg->getDestinationEntity() != getEntityId())
      return;

    if (msg->type == IMC::IoEvent::IOV_TYPE_INPUT_ERROR)
      throw RestartNeeded(msg->error, 5);
  }

  //! Main loop.
  void onMain(void) {
    while (!stopping()) {
      waitForMessages(1.0);
    }
  }
};
} // namespace LiDAR
} // namespace Sensors

DUNE_TASK
