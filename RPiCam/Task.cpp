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
// Author: Alexandre Rocha                                                  *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

#include "Calib.hpp"
#include <cmath>
#include <cstring>

namespace Vision {
//! Insert short task description here.
//!
//! Insert explanation on task behaviour here.
//! @author Alexandre Rocha
using DUNE_NAMESPACES;

namespace RPiCam {
struct Arguments {
  //! Manouver-is-over threshold distance
  float finish_dist;
};

struct Task : public DUNE::Tasks::Task {
  //! LiDAR frontal distance measurement
  double frontal_dist;
  //! Capture RPiCam video
  cv::VideoCapture cap;
  //! Capture width
  uint16_t width;
  //! Capture height
  uint16_t height;
  //! Video frame
  cv::Mat cap_frame;
  //! Undistortion map 1
  cv::Mat map_1;
  //! Undistortion map 2
  cv::Mat map_2;
  //! Filter structuring element
  cv::Mat kernel;
  //! Circle detection parameters
  cv::SimpleBlobDetector::Params params;
  //! Blob algorithm detector object
  cv::Ptr<cv::SimpleBlobDetector> detector;
  //! Detected circles centers vector
  std::vector<cv::KeyPoint> keypoints;

  //! Deviation from center in x-axis
  double delta_x;
  //! Heading reference to aim
  double heading_ref;

  //! Task Arguments
  Arguments m_args;

  //! Constructor.
  //! @param[in] name task name.
  //! @param[in] ctx context.
  Task(const std::string &name, Tasks::Context &ctx)
      : DUNE::Tasks::Task(name, ctx), frontal_dist(0.0), width(640),
        height(480) {
    paramActive(Tasks::Parameter::SCOPE_MANEUVER,
                Tasks::Parameter::VISIBILITY_USER);

    param("Manouver-is-over threshold distance", m_args.finish_dist)
        .defaultValue("1.0")
        .description(
            "Distance used as reference to confirm docking manouver success");

    bind<IMC::Distance>(this);
  }

  void consume(const IMC::Distance *msg) {
    if (!msg->validity)
      return;

    frontal_dist = msg->value;
  }

  //! Update internal state with new parameter values.
  void onUpdateParameters(void) {}

  //! Reserve entity identifiers.
  void onEntityReservation(void) {}

  //! Resolve entity names.
  void onEntityResolution(void) {}

  //! Acquire resources.
  void onResourceAcquisition(void) {

    setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

    cap.open(0);

    if (!cap.isOpened()) {
      inf("Unable to open camera");
      return;
    }
  }

  //! Initialize resources.
  void onResourceInitialization(void) {
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    undistortionMaps(map_1, map_2, cap);

    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));

    // Blob parameters
    params.filterByArea = true;
    params.minArea = 1000;
    params.maxArea = 4 * M_PI * pow(width, 2);

    params.filterByCircularity = true;
    params.minCircularity = 0.8;

    params.filterByConvexity = true;
    params.minConvexity = 0.3;

    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;

    detector = cv::SimpleBlobDetector::create(params);
  }

  //! Release resources.
  void onResourceRelease(void) { 
  
    cap.release(); 
  }

  //! Red Circle Detection
  void redCircleDetection(void) {

    cap.read(cap_frame);

    cv::remap(cap_frame, cap_frame, map_1, map_2, cv::INTER_LINEAR);
    cropROI(cap_frame);

    // Color Detection
    cv::cvtColor(cap_frame, cap_frame, cv::COLOR_BGR2HSV);
    cv::inRange(cap_frame, cv::Scalar(10, 0, 0), cv::Scalar(170, 255, 255),
                cap_frame);

    // Morphologic operations
    cv::GaussianBlur(cap_frame, cap_frame, cv::Size(13, 13), 3);
    cv::morphologyEx(cap_frame, cap_frame, cv::MORPH_CLOSE, kernel);

    // Detect circles
    detector->detect(cap_frame, keypoints);

    // temporary - delete when finished
    drawKeypoints(cap_frame, keypoints, cap_frame, cv::Scalar(0, 0, 255),
                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    for (auto blob_iterator : keypoints) {

      delta_x = blob_iterator.pt.x - cap_frame.cols / 2;

      heading_ref =
          atan(delta_x / cap_frame.cols * tan(MAX_PICAM_ANGLE * M_PI / 180));
    }

    cv::imshow("debug window", cap_frame);

    cv::waitKey(1000);
  }

  //! Main loop.
  void onMain(void) {

    while (!stopping()) {

      redCircleDetection();

      waitForMessages(1.0);
    }
  }
};
} // namespace RPiCam
} // namespace Vision

DUNE_TASK