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
Mat gray;
Mat dest;
Mat mask;


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
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

	vcap.read(img);
	if (img.empty()) {
		cerr << "ERROR! blank frame grabbed\n";
		return 0;
	}
	//string path="/home/aswath/Capstone_Project/catkin_ws/src/nxtbot/assets/road.jpeg";
	//string path = "road.jpeg";
	//img = imread(path);
	
	Size sz = img.size();

	if (img.cols > 500 || img.rows > 500) {
		//resize(img, img, Size(0, 0), 0.5, 0.5);
	}

	imshow("Input image", img);
	cout << "Size of input image: " << img.size() << endl;
	cout << "Size of input img.height" << img.rows << endl;
	cout << "Size of input img.width" << img.cols << endl;

	Mat im_out(img.rows, img.cols, CV_8UC1);
	Mat mask_out(img.rows, img.cols, CV_8UC1);
	cout << "Size of Bird's eye view image: " << im_out.size() << endl;

	int min_region_area = int(min_region_area_factor * img.cols * img.rows);
	uchar padding = 1;
	dest = Mat::zeros(img.rows, img.cols, CV_8UC1);
	mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

	int x = img.cols / 2, y = img.rows - 20;
	grow(img, dest, mask, Point(x, y), 100);
	
	dest = dest + mask * padding;
	mask = mask * 255;
	imshow("mask", mask);

	vector<Point2f> src;
	src.push_back(Point2f(200, 310));
	src.push_back(Point2f(285, 265));
	src.push_back(Point2f(400, 265));
	src.push_back(Point2f(475, 310));

	int row_offset = img.rows - 310;
	int mid_align_offset = img.cols/2;
	int zoom_out_factor = 1;

	vector<Point2f> dst;
	dst.push_back(Point2f((200 + mid_align_offset), 310 + row_offset));
	dst.push_back(Point2f((200 + mid_align_offset), 286));
	dst.push_back(Point2f((475 + mid_align_offset), 339));
	dst.push_back(Point2f((475 + mid_align_offset), 310 + row_offset));

	bool calibrate=true;
	
	Mat H = findHomography(src, dst);
	cout << "H:" << H << endl;
	
	/*
	cv::Mat_<double> H(3,3);
	H<< -0.4329863893783613, -5.854818923942802, 800.014098472485,
	-0.08703559309736916, -6.912913870558576, 600.452587986193,
	-7.955721489704677e-05, -0.004179839334454694, 1;
	*/

	cuda::GpuMat gpuimg = cuda::GpuMat(img);
	cuda::GpuMat gpuim_out = cuda::GpuMat(img);
	cuda::GpuMat gpumask = cuda::GpuMat(mask);
	cuda::GpuMat gpumask_out = cuda::GpuMat(mask);

	cuda::warpPerspective(gpuimg, gpuim_out, H, im_out.size(), INTER_LINEAR, BORDER_CONSTANT,0);
   	cuda::warpPerspective(gpumask, gpumask_out, H, mask_out.size());

	gpuim_out.download(im_out);
	gpumask_out.download(mask_out);
	namedWindow("Output", 1);
	resize(im_out, im_out, img.size());
	imshow("Output", im_out);
	namedWindow("Mask output", 1);
	resize(mask_out, mask_out, mask_out.size());
	imshow("Mask output", mask_out);
	setMouseCallback("Output", mouse_callback);
	imwrite("Output.jpg", im_out);

	cv::threshold(mask_out, gray, 1,255,THRESH_BINARY);
	imshow("Threshold", gray);
	waitKey(0);

    octomap::OcTree tree(0.01); // create empty tree with resolution 0.1
                        // insert some measurements of occupied cells
	int val=0;
    for (int x = 0; x < gray.rows; x++)
    {
        for (int y = 0; y < gray.cols; y++)
        {
			
            point3d endpoint((float)x * 0.01f, (float)y * 0.01f, 0.0f);
			val=gray.at<char>(x,y);
			//cout<<val <<" x " << x<<" y: "<<y<<"\t";
			//Vec3b bgrPixel = mask_out.at<Vec3b>(x, y);
			if (val<0){
            	tree.updateNode(endpoint, false); // integrate 'occupied' measurement
			}else{
				tree.updateNode(endpoint, true); // integrate 'occupied' measurement
			}
        }
    }

    ros::init(argc, argv, "octomap_trial");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    ros::Publisher pub_octomap = n.advertise<octomap_msgs::Octomap>("my_octomap", 1);

    octomap_msgs::Octomap octomap;
	octomap.header.frame_id = "my_frame";
	//rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10 
    
	//OcTree myOctomap("simple_tree.bt");

	while (ros::ok()){
		
        vcap.read(img);
        if (img.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            continue;
        }
		Mat dest = Mat::zeros(img.rows, img.cols, CV_8UC1);
		Mat mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

		grow(img, dest, mask, Point(x, y), 100);
		
		gpumask = cuda::GpuMat(mask);
		gpumask_out = cuda::GpuMat(mask);
		cuda::warpPerspective(gpumask, gpumask_out, H, mask_out.size());
		gpumask_out.download(mask_out);

		cv::threshold(mask_out, gray, 1,255,THRESH_BINARY);
		imshow("Mask output", gray);

		int val=0;
		for (int x = 0; x < gray.rows; x++)
		{
			for (int y = 0; y < gray.cols; y++)
			{
				point3d endpoint((float)x * 0.01f, (float)y * 0.01f, 0.0f);
				val=gray.at<char>(x,y);
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