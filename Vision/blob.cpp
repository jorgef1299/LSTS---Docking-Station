#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

  VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
  namedWindow("window1");

  Mat cap_frame;

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Filter by Color
    params.filterByColor = true;
    params.blobColor = 10;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.5;

  while (true) {
    cap.read(cap_frame);

    // Read image
    cvtColor(cap_frame, cap_frame, COLOR_BGR2GRAY); 

    // Detect blobs.
    vector<KeyPoint> keypoints;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    detector->detect(cap_frame, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints;
    drawKeypoints( cap_frame, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Show blobs
    imshow("window1", im_with_keypoints );

    if ((char)waitKey(30) == 27)
      break;
  }
  return 0;
}