#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

float cam_matrix_data[9] = {
    681.9474487304688,    0.000000000000000000, 279.387553359592862,
    0.000000000000000000, 679.4100341796875,    240.90015807337113,
    0.000000000000000000, 0.000000000000000000, 1.000000000000000000};

float dist_matrix_data[5] = {-0.4624503562479969, -0.43432558990654135,
                             0.001974482671297278, -0.008023538703377298,
                             2.9986277588121113};

int roi[4] = {7, 13, 623, 453};

void crop_ROI(Mat &frame) {

  Rect new_roi = Rect(roi[0], roi[1], roi[2], roi[3]);
  frame = frame(new_roi);
}

int main() {

  int width = 640;
  int height = 480;

  Mat cap_frame, calib_frame;
  Mat camera_matrix, dist_coefs;

  camera_matrix = Mat(3, 3, CV_32F, cam_matrix_data);
  dist_coefs = Mat(1, 5, CV_32F, dist_matrix_data);

  VideoCapture cap(0);

  if (!cap.open(0)) {
    cout << "Error Opening Camera" << endl;
    return 0;
  }

  cap.set(CAP_PROP_FRAME_WIDTH, width);
  cap.set(CAP_PROP_FRAME_HEIGHT, height);

  cap.read(cap_frame);
  Mat map1(cap_frame.size(), CV_32F);
  Mat map2(cap_frame.size(), CV_32F);

  initUndistortRectifyMap(camera_matrix, dist_coefs, Mat(), camera_matrix,
                          cap_frame.size(), CV_32F, map1, map2);

  while (true) {
    cap.read(cap_frame);

    calib_frame = cap_frame.clone();

    // UNDISTORTION - to use, comment initUndistortRectifyMap() and Remap() functions
    // undistort(cap_frame, calib_frame, camera_matrix, dist_coefs);

    // REMAP
    remap(cap_frame, calib_frame, map1, map2, INTER_LINEAR);

    crop_ROI(calib_frame);

    imshow("Original Frame", cap_frame);
    imshow("Calib Frame", calib_frame);

    if ((char)waitKey(30) == 27)
      break;
  }

  cap.release();
}
