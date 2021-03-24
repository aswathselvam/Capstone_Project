#include <opencv4/opencv2/cudaarithm.hpp>
#include <opencv4/opencv2/cudafilters.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/cuda.hpp>
//#include <cuda_runtime.h>
//#include <cuda_profiler_api.h>
#include <opencv4/opencv2/cudawarping.hpp>
#include <opencv4/opencv2/videoio.hpp>

#include <iostream>

using namespace cv;
using namespace std;
	
int main(){
    Mat frame;
    const std::string videoStreamAddress = "http://192.168.29.132:8080/video";

    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open("http://192.168.29.132:8080/video", apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);
        if (waitKey(5) >= 0)
            break;
    }
}