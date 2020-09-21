#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

using namespace cv;

int main(int argc, char* argv[])
{
    int low_H = 10, low_S = 0, low_V = 0;
    int high_H = 170, high_S = 255, high_V = 255;
    const String window_detection_name = "Object Detection";

    VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
    namedWindow(window_detection_name);

    Mat frame, frame_HSV, frame_threshold, edges;

    Mat opening_kernel = getStructuringElement( MORPH_RECT , Size(10, 10));
    Mat closing_kernel = getStructuringElement( MORPH_RECT , Size(10, 10));

    while (true) {
        cap >> frame;
        if(frame.empty())
        {
            break;
        }
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        frame_threshold=~frame_threshold;

        // Show the frames / Smoothening & Filtering
        // Median Smothenigng
        medianBlur(frame_threshold, frame_threshold, 3);
        
        // Opening and Closing filtering - closing maybe not helping
        morphologyEx(frame_threshold, frame_threshold, MORPH_OPEN, opening_kernel);
        // morphologyEx(frame_threshold, frame_threshold, MORPH_CLOSE, closing_kernel);
        
        // Others 
        // Canny(frame_threshold,edges,50,200);

        imshow(window_detection_name, frame_threshold);
        
        
        if ((char)waitKey(30) == 27)
            break;
    }
    return 0;
}