#include <cmath>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define MAX_PICAM_ANGLE 31.1

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

void rectifying_maps(Mat &map1, Mat &map2, VideoCapture &cap) {
  Mat temp;
  Mat camera_matrix, dist_coefs;

  cap.read(temp);
  map1 = Mat(temp.size(), CV_32F);
  map2 = Mat(temp.size(), CV_32F);

  camera_matrix = Mat(3, 3, CV_32F, cam_matrix_data);
  dist_coefs = Mat(1, 5, CV_32F, dist_matrix_data);

  initUndistortRectifyMap(camera_matrix, dist_coefs, Mat(), camera_matrix,
                          temp.size(), CV_32F, map1, map2);
}

int main(int argc, char *argv[]) {

  int width = 640, height = 480;
  bool first_loop = true;
  double alpha;

  Mat cap_frame, calib_frame, im_with_keypoints;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));

  Mat map1, map2;

  SimpleBlobDetector::Params params;

  // Area filtering parameters
  params.filterByArea = true;
  params.minArea = 1000;
  params.maxArea = 4 * M_PI * pow(width, 2);

  // Circularity filtering parameters (1=true circle)
  params.filterByCircularity = true;
  params.minCircularity = 0.8;

  // Convexity filtering parameters
  params.filterByConvexity = true;
  params.minConvexity = 0.3;

  // Inertia filtering parameters
  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;

  vector<KeyPoint> keypoints;
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

  VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);

  if (!cap.open(0)) {
    cout << "Error Opening Camera" << endl;
    return 0;
  }

  cap.set(CAP_PROP_FRAME_WIDTH, width);
  cap.set(CAP_PROP_FRAME_HEIGHT, height);

  rectifying_maps(map1, map2, cap);

  while (true) {

    cap.read(cap_frame);
    calib_frame = cap_frame.clone();

    remap(cap_frame, calib_frame, map1, map2, INTER_LINEAR);

    crop_ROI(calib_frame);

    cvtColor(calib_frame, calib_frame, COLOR_BGR2HSV);

    // Detect the object based on HSV Range Values (returns monochromatic image)
    inRange(calib_frame, Scalar(10, 0, 0), Scalar(170, 255, 255), calib_frame);

    GaussianBlur(calib_frame, calib_frame, Size(13, 13), 3);
    morphologyEx(calib_frame, calib_frame, MORPH_CLOSE, kernel);

    detector->detect(calib_frame, keypoints);

    // Draw detected blobs as red circles.
    drawKeypoints(calib_frame, keypoints, im_with_keypoints, Scalar(0, 0, 255),
                  DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    for (auto blobIterator : keypoints) {
      alpha = atan((blobIterator.pt.x - calib_frame.cols / 2) /
                   calib_frame.cols * tan(MAX_PICAM_ANGLE * M_PI / 180));

      cout << "Distance from x center: "
           << blobIterator.pt.x - calib_frame.cols / 2 << '\n'
           << "The vehicle must turn: " << alpha << '\n'
           << endl;
    }

    imshow("Blob Detection", im_with_keypoints);

    if ((char)waitKey(30) == 27)
      break;
  }

  return 0;
}