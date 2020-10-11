#include <iostream>
#include <cmath>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

#define MAX_PICAM_ANGLE 31.1

void use_only_ROI(Mat &frame_src){

  int x = 0;
  int y = frame_src.rows / 4;
  int w = frame_src.cols;
  int h = frame_src.rows / 2;

  Rect region_of_interest = Rect(x, y, w, h);
  frame_src = frame_src(region_of_interest);
}

int main(int argc, char *argv[]) {

  bool first_loop = true;
  double alpha;

  VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);

  Mat cap_frame, im_with_keypoints, blob_image;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(10, 10));
 
  // Setup SimpleBlobDetector parameters.
  SimpleBlobDetector::Params params;

  // Set Area filtering parameters
  params.filterByArea = true;
  params.minArea = 1000;

  cap.read(cap_frame);
  params.maxArea = 4*M_PI*pow(cap_frame.rows/4, 2); //Max Blob Area, considerind ROI

  // Set Circularity filtering parameters (1=true circle)
  params.filterByCircularity = true;
  params.minCircularity = 0.8;

  // Set Convexity filtering parameters
  params.filterByConvexity = true;
  params.minConvexity = 0.3;

  // Set inertia filtering parameters
  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;

  // Detect blobs.
  vector<KeyPoint> keypoints;
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

  while (true) {
    cap.read(cap_frame);
    use_only_ROI(cap_frame);
    
    // Convert from RGB to HSV colorspace
    cvtColor(cap_frame, cap_frame, COLOR_BGR2HSV);

    // Detect the object based on HSV Range Values (returns monochromatic image)
    inRange(cap_frame, Scalar(10, 0, 0),
            Scalar(170, 255, 255), cap_frame);

    GaussianBlur(cap_frame, cap_frame, Size(7, 7), 1);
    morphologyEx(cap_frame, cap_frame, MORPH_CLOSE, kernel); 

    detector->detect(cap_frame, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle
    // corresponds to the size of blob
    drawKeypoints(cap_frame, keypoints, im_with_keypoints, Scalar(0, 0, 255),
                  DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

   for(auto blobIterator : keypoints){
      alpha = atan((blobIterator.pt.x - cap_frame.cols/2)/cap_frame.cols * tan(MAX_PICAM_ANGLE * M_PI/180));

      cout << "Distance from x center: " << blobIterator.pt.x - cap_frame.cols/2 << '\n'
           << "The vehicle must turn: " << alpha << '\n' << endl;
    }
    
    imshow("Blob Detection", im_with_keypoints);
    // imshow("Blob Detection", cap_frame);

    if ((char)waitKey(30) == 27)
      break;
  }
  return 0;
}