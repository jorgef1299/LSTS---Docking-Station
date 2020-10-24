#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

float cam_matrix_data[9] = {
  506.7820129394531,0.000000000000000000,331.2240127614132,
  0.000000000000000000,511.05706787109375,247.3048612986986,
  0.000000000000000000,0.000000000000000000,1.000000000000000000
};

float dist_matrix_data[5] = {
  0.12114983560894715,
  -0.4565965457549959,
  0.005491031189558299,
  -0.0016131624426953335,
  0.1000164061597219
};

int roi[4] = {
  28,
  25,
  590,
  438
};

void crop_ROI(Mat &frame){

  Rect new_roi = Rect(roi[0], roi[1], roi[2], roi[3]);
  frame = frame(new_roi);
}

int main(){

  int width = 640;
  int height = 480;

  Mat cap_frame, calib_frame;
  Mat camera_matrix, dist_coefs;

  camera_matrix = Mat(3, 3, CV_32F, cam_matrix_data);
  dist_coefs = Mat(1, 5, CV_32F, dist_matrix_data);
  
  VideoCapture cap(0);
  
  if(!cap.open(0)){
    cout << "Error Opening Camera" << endl;
    return 0;
  }
  
  cap.set(CAP_PROP_FRAME_WIDTH, width);
  cap.set(CAP_PROP_FRAME_HEIGHT, height);

  cap.read(cap_frame);
  Mat map1(cap_frame.size(), CV_32F);
  Mat map2(cap_frame.size(), CV_32F);

  initUndistortRectifyMap(camera_matrix, dist_coefs, Mat(), 
          camera_matrix, cap_frame.size(), CV_32F, map1, map2);
  
  while (true) {
    cap.read(cap_frame);

    calib_frame = cap_frame.clone();
    
    // UNDISTORTION
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