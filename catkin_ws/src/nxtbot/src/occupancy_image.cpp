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

#include <iostream>
#include <cassert>
#include <vector>
#include <stack>

using namespace std;
using namespace cv;

Point pt(-1, -1);
int threshold = 200;
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

int main() {
    string path="/home/aswath/Capstone_Project/catkin_ws/src/nxtbot/assets/road.jpeg";
	//string path = "road.jpeg";
	Mat img = imread(path);
	Size sz = img.size();

	if (img.cols > 500 || img.rows > 500) {
		//resize(img, img, Size(0, 0), 0.5, 0.5);
	}

	imshow("Input image", img);
	cout << "Size of input image: " << img.size() << endl;
	int rows = sz.height;
	int cols = sz.width;
	cout << "Size of input img.height" << rows << endl;
	cout << "Size of input img.width" << cols << endl;

	int sfr = 3;
	int sfc = 4;
	int out_im_rows = sfr * rows;
	int out_im_cols = sfc * cols;

	Mat im_out(out_im_rows, out_im_cols, CV_8UC1);
	Mat mask_out(out_im_rows, out_im_cols, CV_8UC1);
	cout << "Size of Bird's eye view image: " << im_out.size() << endl;

	int min_region_area = int(min_region_area_factor * img.cols * img.rows);
	uchar padding = 1;
	Mat dest = Mat::zeros(img.rows, img.cols, CV_8UC1);
	Mat mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

	int x = img.cols / 2, y = img.rows - 50;
	if (dest.at<uchar>(Point(x, y)) == 0) {
		grow(img, dest, mask, Point(x, y), 200);

		int mask_area = (int)sum(mask).val[0];
		if (mask_area > min_region_area) {
			dest = dest + mask * padding;
			mask = mask * 255;
			imshow("mask", mask);
			//waitKey(0);
			cout << "Mask.size() " << mask.size() << endl;
			if (++padding > max_region_num) { cout << "run out of max_region"; return -1; }
		}
		else dest = dest + mask * 255;
		//mask -= mask;
	}


	vector<Point2f> src;
	src.push_back(Point2f(200, 310));
	src.push_back(Point2f(285, 265));
	src.push_back(Point2f(400, 265));
	src.push_back(Point2f(475, 310));



	int row_offset = out_im_rows - rows;
	int mid_align_offset = (out_im_cols / 2) - (cols / 2);
	int zoom_out_factor = 1;

	vector<Point2f> dst;
	dst.push_back(Point2f((200 + mid_align_offset), 310 + row_offset));
	dst.push_back(Point2f((200 + mid_align_offset), 286));
	dst.push_back(Point2f((475 + mid_align_offset), 339));
	dst.push_back(Point2f((475 + mid_align_offset), 310 + row_offset));

	Mat H = findHomography(src, dst);
	cout << "H:" << H << endl;

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
	resize(mask_out, mask_out, img.size());
	imshow("Mask output", mask_out);
	setMouseCallback("Output", mouse_callback);
	imwrite("Output.jpg", im_out);
	waitKey(0);
	return 0;
}