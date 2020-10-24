#include <iostream>
#include <cmath>

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

#define MAX_PICAM_ANGLE 31.1

int main(int argc, char *argv[]) {

  bool first_loop = true;
  double alpha;

  VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);

  Mat cap_frame, im_with_keypoints, blob_image;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));

  // Setup SimpleBlobDetector parameters.
  SimpleBlobDetector::Params params;

  // Set Area filtering parameters
  params.filterByArea = true;
  params.minArea = 1000;

  cap.read(cap_frame);
  params.maxArea = 4*M_PI*pow(cap_frame.rows, 2); //Max Blob Area

  // Set Circularity filtering parameters (1=true circle)
  params.filterByCircularity = true;
  params.minCircularity = 0.8;

  // Set Convexity filtering parameters
  params.filterByConvexity = true;
  params.minConvexity = 0.3;

  // Set inertia filtering parameters
  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;

  vector<KeyPoint> keypoints;
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

  while (true) {
    
    cap.read(cap_frame);

    cvtColor(cap_frame, cap_frame, COLOR_BGR2HSV);

    // Detect the object based on HSV Range Values (returns monochromatic image)
    inRange(cap_frame, Scalar(5, 0, 0),
            Scalar(175, 255, 255), cap_frame);

    GaussianBlur(cap_frame, cap_frame, Size(13, 13), 3);
    morphologyEx(cap_frame, cap_frame, MORPH_CLOSE, kernel); 

    detector->detect(cap_frame, keypoints);

    // Draw detected blobs as red circles.
    drawKeypoints(cap_frame, keypoints, im_with_keypoints, Scalar(0, 0, 255),
                  DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

   for(auto blobIterator : keypoints){
      alpha = atan((blobIterator.pt.x - cap_frame.cols/2)/cap_frame.cols * tan(MAX_PICAM_ANGLE * M_PI/180));

      cout << "Distance from x center: " << blobIterator.pt.x - cap_frame.cols/2 << '\n'
           << "The vehicle must turn: " << alpha << '\n' << endl;
    }
    
    imshow("Blob Detection", im_with_keypoints);

    if ((char)waitKey(30) == 27)
      break;
  }
  return 0;
}