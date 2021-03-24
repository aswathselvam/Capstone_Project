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

#include "ros/ros.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
//https://gitlab.tu-berlin.de/breakdowncookie/ORB_SLAM2/blob/987e93d81ce3c837be23990fcf62f1dc539efc70/orb_slam2_ros/src/ROSPublisher.cpp
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <iostream>
#include <cassert>
#include <vector>
#include <stack>
#include <cstdlib>

using namespace std;
using namespace cv;
using namespace octomap;

Point pt(-1, -1);
int threshold = 20;
const uchar max_region_num = 100;
const double min_region_area_factor = 0.0;
const Point PointShift2D[8] = {
	Point(1, 0),
	Point(1, -1),
	Point(0, -1),
	Point(-1, -1),
	Point(-1, 0),
	Point(-1, 1),
	Point(0, 1),
	Point(1, 1)
};
cuda::GpuMat gpumask;
cuda::GpuMat gpumask_out;
Mat binary;
Mat dest;
Mat mask;
double ppcm_width = 0.0377622, ppcm_height=0.0487805;



void mouse_callback(int event, int x, int y, int flag, void* param) {
	if (event == EVENT_LBUTTONDOWN) {
		pt.x = x;
		pt.y = y;
		cout << pt.x << " " << pt.y << endl;
	}
}

void grow(cv::Mat& src, cv::Mat& dest, cv::Mat& mask, cv::Point seed, int threshold) {

	stack<cv::Point> point_stack;
	point_stack.push(seed);

	while (!point_stack.empty()) {
		cv::Point center = point_stack.top();
		mask.at<uchar>(center) = 1;
		point_stack.pop();

		for (int i = 0; i < 8; ++i) {
			cv::Point estimating_point = center + PointShift2D[i];
			if (estimating_point.x < 0
				|| estimating_point.x > src.cols - 1
				|| estimating_point.y < 0
				|| estimating_point.y > src.rows - 1) {

				continue;
			}
			else {

				int delta = int(pow(src.at<cv::Vec3b>(center)[0] - src.at<cv::Vec3b>(estimating_point)[0], 2)
					+ pow(src.at<cv::Vec3b>(center)[1] - src.at<cv::Vec3b>(estimating_point)[1], 2)
					+ pow(src.at<cv::Vec3b>(center)[2] - src.at<cv::Vec3b>(estimating_point)[2], 2));
				if (dest.at<uchar>(estimating_point) == 0
					&& mask.at<uchar>(estimating_point) == 0
					&& delta < threshold) {
					mask.at<uchar>(estimating_point) = 1;
					point_stack.push(estimating_point);
				}
			}
		}
	}
}

int main(int argc, char **argv) {
	Mat img;

	cv::VideoCapture vcap;
	//open the video stream and make sure it's opened
    
	vcap.open("http://192.168.29.132:8080/video", cv::CAP_ANY);
    // check if we succeeded
    if (!vcap.isOpened()) {
        cout << "ERROR! Unable to open camera\n";
        return -1;
    }

	vcap.read(img);
	if (img.empty()) {
		cout << "ERROR! blank frame grabbed\n";
		return 0;
	}
	dest = Mat::zeros(img.rows, img.cols, CV_8UC1);
	mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

	int x = img.cols / 2, y = img.rows - 20;

	cv::Mat_<double> H(3,3);

	H<< -0.3533094995479479, -1.770444793641235, 557.3953069956937,
 	0.1624698020692046, -3.595786418237763, 1115.419594647802,
	0.0002770977906833813, -0.00456378934828412, 1;

    octomap::OcTree tree(0.01); // create empty tree with resolution 0.1

    ros::init(argc, argv, "octomap_trial");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    ros::Publisher pub_octomap = n.advertise<octomap_msgs::Octomap>("my_octomap", 1);

    octomap_msgs::Octomap octomap;
	octomap.header.frame_id = "my_frame";
	//rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10 
    
	//OcTree myOctomap("simple_tree.bt");

	while (ros::ok()){	
		for(int i = 0; i < 50; i++) {
			vcap.grab();
		}	

        vcap.read(img);
        if (img.empty()) {
            cout << "ERROR! blank frame grabbed\n";
            continue;
        }

		Mat dest = Mat::zeros(img.rows, img.cols, CV_8UC1);
		Mat mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

		grow(img, dest, mask, Point(x, y), 20);
		mask = mask * 255;
	    Mat Homo;
		warpPerspective(img, Homo, H, img.size());
		imshow("Mask", Homo);

		gpumask_out = cuda::GpuMat(mask);
		cuda::warpPerspective(cuda::GpuMat(mask), gpumask_out, H, mask.size());		
		gpumask_out.download(mask);
		imshow("Perspective", mask);
		
		//cv::threshold(mask, binary, 150,255,THRESH_BINARY);
		binary = mask;
		imshow("Mask output", binary);
		waitKey(10);

		int val=0;
		for (int x = 0; x < binary.rows; x++)
		{
			for (int y = 0; y < binary.cols; y++)
			{
				octomap::point3d endpoint((float)x * 0.01f* ppcm_width, (float)y * 0.01f * ppcm_height, 0.0f);
				val=binary.at<char>(x,y);
				//cout<<val <<" x " << x<<" y: "<<y<<"\t";
				//Vec3b bgrPixel = mask_out.at<Vec3b>(x, y);
				if (val<0){
					tree.updateNode(endpoint, false); // integrate 'occupied' measurement
				}else{
					tree.updateNode(endpoint, true); // integrate 'occupied' measurement
				}
			}
		}
		if(octomap_msgs::binaryMapToMsg(tree, octomap)|| 1==1){
			octomap.header.stamp = ros::Time::now();
			pub_octomap.publish(octomap);
			ros::spinOnce();
			loop_rate.sleep();
		}

    }


}