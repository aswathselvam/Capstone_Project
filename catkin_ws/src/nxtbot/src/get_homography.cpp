#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;

Point2f dst_vertices[4];
Point2f src_vertices[4];
int i;


void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst){
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

void DrawRectangle(Mat& img, Rect box)
{
	//Draw a rectangle with random color
	rectangle(img, box.tl(), box.br(), Scalar(g_rng.uniform(0, 255),
					g_rng.uniform(0,255),g_rng.uniform(0,255)));
}

void mouse_callback(int event, int x, int y, int flag, void* param) {
	if (event == EVENT_LBUTTONDOWN) {
        if(i<4){
            std::cout << "Choosing source points " << i << " "<< Point(x,y) << std::endl;
            src_vertices[i] = Point(x,y);
        }else if(i<8){
            std::cout << "Choosing destination points: " << i-4 << " " << Point(x,y) << std::endl;
            dst_vertices[i-4] = Point(x,y);
        }else{
            std::cout << "Done choosing points "<< std::endl;
            return;
        }
        ++i;
		//std::cout << x << " " << y << std::endl;
	}
}

int main(){
    Mat frame;
    VideoCapture cap;

    cap.open("http://192.168.29.132:8080/video", cv::CAP_ANY);
    // check if we succeeded
    if (!cap.isOpened()) {
        std::cout << "ERROR! Unable to open camera\n";
        return -1;
    }


    cap.read(frame);
    // check if we succeeded
    if (frame.empty()) {
        std::cout << "ERROR! blank frame grabbed\n";
        return 0;
    }
    Mat src = frame;
    imshow("Src", frame);

    i=0;
	setMouseCallback("Src", mouse_callback);
    waitKey();

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    Mat dst(src.rows, src.cols, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    Mat dst2(src.rows, src.cols, CV_8UC3);
    transform(src_vertices, dst_vertices, src, dst2);


    imshow("src", src);
    imshow("dst", dst);
    imshow("dst2", dst2);
    waitKey();
}