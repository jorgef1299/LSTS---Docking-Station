#include <iostream>
#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

Mat rgb2hsv_filtering(Mat &frame_src){
  Mat frame_final;

  int low_h = 10, low_s = 0, low_v = 0;
  int high_h = 170, high_s = 255, high_v = 255;

  // Kernel for opening operation - adjust 2nd argument
  Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));

  // Convert from RGB to HSV colorspace
  cvtColor(frame_src, frame_final, COLOR_BGR2HSV);

  // Detect the object based on HSV Range Values
  inRange(frame_final, Scalar(low_h, low_s, low_v),
          Scalar(high_h, high_s, high_v), frame_final);
          
  frame_final = frame_final;

  // Median Smoothening
  //medianBlur(frame_final, frame_final, 3);

  // Opening
  morphologyEx(frame_final, frame_final, MORPH_OPEN, kernel);
  morphologyEx(frame_final, frame_final, MORPH_GRADIENT, kernel);

  return frame_final;
}

Mat hsv_circle_detection(Mat &frame_src){
  Mat frame_final;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(10, 10));

  cvtColor(frame_src, frame_final, COLOR_BGR2GRAY); 
  
  GaussianBlur(frame_final, frame_final, Size(5, 5), 1);
  Canny(frame_final, frame_final, 50, 100);
  erode(frame_final, frame_final,(10,10));
  dilate(frame_final, frame_final,(10,10));
  morphologyEx(frame_final, frame_final, MORPH_CLOSE, kernel);

  vector<Vec3f> circles;
  HoughCircles(
      frame_final, circles, HOUGH_GRADIENT, 1,
      frame_final.rows / 4, 100, 40, 10, 500);
  
  for (size_t i = 0; i < circles.size(); i++) {
    Vec3i c = circles[i];
    Point center = Point(c[0], c[1]);
    // circle center
    circle(frame_final, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
    // circle outline
    int radius = c[2];
    circle(frame_final, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
  }

  return frame_final;
}


int main(int argc, char *argv[]) {

  const String window_one_name = "HSV Filtering";
  const String window_two_name = "Circle Detection";

  VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
  namedWindow(window_one_name);
  namedWindow(window_two_name);

  Mat frame_rgb, frame_hsv, frame_gray;

  Mat opening_kernel = getStructuringElement(MORPH_RECT, Size(20, 20));

  while (true) {
    cap.read(frame_rgb);

    frame_hsv = rgb2hsv_filtering(frame_rgb);

    frame_gray = hsv_circle_detection(frame_rgb);
    
    imshow(window_one_name, frame_hsv);
    imshow(window_two_name, frame_gray);

    if ((char)waitKey(30) == 27)
      break;
  }
  return 0;
}