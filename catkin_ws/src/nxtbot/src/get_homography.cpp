#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;

Point2f dst_vertices[4];
Point2f src_vertices[4];
int i;
Rect g_rectangle;
bool g_bDrawingBox = false;

const double CC_WIDTH=5.4, CC_LENGTH=8;
double ppcm_width, ppcm_height;

void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst){
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

void DrawRectangle(Mat& img, Rect box)
{
	//Draw a rectangle with random color
	rectangle(img, box.tl(), box.br(), Scalar(100,150,100));
}

void mouse_callback(int event, int x, int y, int flag, void* param) {
    Mat& image = *(cv::Mat*) param;
    switch (event) {
	case EVENT_MOUSEMOVE: {    // When mouse moves, get the current rectangle's width and height
		if (g_bDrawingBox) {
			g_rectangle.width = x - g_rectangle.x;
			g_rectangle.height = y - g_rectangle.y;
		}
	}
    break;

	case EVENT_LBUTTONDOWN: {
        if(i<4){
            std::cout << "Choosing source points " << i << " "<< Point(x,y) << std::endl;
            src_vertices[i] = Point(x,y);
            ++i;
        }else if(i<8){
            g_bDrawingBox = true;
		    g_rectangle = Rect(x, y, 0, 0);
        }else{
            std::cout << "Done choosing points "<< std::endl;
            return;
        }
	}
    break; 

    case EVENT_LBUTTONUP: {   //when the left mouse button is released
        if(g_bDrawingBox){
            if (g_rectangle.width < 0) {
                g_rectangle.x += g_rectangle.width;
                g_rectangle.width *= -1;
            }

            if (g_rectangle.height < 0) {
                g_rectangle.y += g_rectangle.height;
                g_rectangle.height *= -1;
            }
            Rect r = g_rectangle;
            
            dst_vertices[0] = Point(r.x, r.y + r.height);
            dst_vertices[1] = Point(r.x, r.y);
            dst_vertices[2] = Point(r.x + r.width, r.y);
            dst_vertices[3] = Point(r.x + r.width, r.y + r.height);
            std::cout << "Choosing destination points: " << dst_vertices[0] << std::endl;
            std::cout << "Choosing destination points: " << dst_vertices[1] << std::endl;
            std::cout << "Choosing destination points: " << dst_vertices[2] << std::endl;
            std::cout << "Choosing destination points: " << dst_vertices[3] << std::endl;
            std::cout << "Choosing destination points: " << dst_vertices[3] << std::endl;
            std::cout << r << std::endl;
            ppcm_width = CC_WIDTH/r.width;
            ppcm_height = CC_LENGTH/r.height;
            DrawRectangle(image, g_rectangle);
            g_bDrawingBox = false;
        }

	}
	break;

    }
}

int main(){
    Mat src;
    VideoCapture cap;

    cap.open("http://192.168.29.132:8080/video", cv::CAP_ANY);
    // check if we succeeded
    if (!cap.isOpened()) {
        std::cout << "ERROR! Unable to open camera\n";
        return -1;
    }


    cap.read(src);
    // check if we succeeded
    if (src.empty()) {
        std::cout << "ERROR! blank frame grabbed\n";
        return 0;
    }

    std::cout<<src.rows <<" cols: " << src.cols<<std::endl;
    imshow("Src", src);
	Mat tempImage;

    i=0;
	setMouseCallback("Src", mouse_callback,(void*) &src);
	while (1) {
        cap.read(src);
		src.copyTo(tempImage);
		if (g_bDrawingBox)
			DrawRectangle(tempImage, g_rectangle);
		imshow("Src", tempImage);
		if (waitKey(10) == 27)  // stop drawing rectanglge if the key is 'ESC'
			break;
	}
    std::cout<<"ppcm width: "<<ppcm_width <<" ppcm_height: " <<ppcm_height<<std::endl; 

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    std::cout<<"Perspective Transform Matrix: "<<M<<std::endl;
    Mat dst(src.rows, src.cols, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    Mat dst2(src.rows, src.cols, CV_8UC3);
    transform(src_vertices, dst_vertices, src, dst2);


    imshow("src", src);
    imshow("dst", dst);
    imshow("dst2", dst2);
    waitKey();
}