#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**)
{
    VideoCapture video_capture(0); // open the default camera
    if(!video_capture.isOpened())  // check if we succeeded
        return -1;

    Mat gray_image;
    namedWindow("edges",1);
    while(true)
    {
        Mat frame;
        video_capture >> frame; // get a new frame from camera
        cvtColor(frame, gray_image, COLOR_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        imshow("gray image", gray_image);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}