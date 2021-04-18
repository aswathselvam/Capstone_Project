#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace std;
using namespace cv;

Point2f dst_vertices[4];
Point2f src_vertices[4];
int i;
Rect g_rectangle;
bool g_bDrawingBox = false;

const double CC_WIDTH = 18, CC_LENGTH = 18;
double ppcm_width, ppcm_height;


void DrawRectangle(Mat& img, Rect box)
{
    //Draw a rectangle with random color
    rectangle(img, box.tl(), box.br(), Scalar(100, 150, 100));
}
void CallBackFn(int event,int x,int y,int flags,void *userdata) {
    if(event==EVENT_LBUTTONDOWN){
        cout<<"("<<x<<","<<y<<")"<<endl;
    }
}
void mouse_callback(int event, int x, int y, int flag, void* param) {
    Mat& image = *(cv::Mat*)param;
    switch (event) {
    case EVENT_MOUSEMOVE: {    // When mouse moves, get the current rectangle's width and height
        if (g_bDrawingBox) {
            g_rectangle.width = x - g_rectangle.x;
            g_rectangle.height = y - g_rectangle.y;
        }
    }
                        break;

    case EVENT_LBUTTONDOWN: {
        if (i < 4) {
            std::cout << "Choosing source points " << i << " " << Point(x, y) << std::endl;
            src_vertices[i] = Point(x, y);
            ++i;
        }
        else if (i < 8) {
            g_bDrawingBox = true;
            g_rectangle = Rect(x, y, 0, 0);
        }
        else {
            std::cout << "Done choosing points " << std::endl;
            return;
        }
    }
                          break;

    case EVENT_LBUTTONUP: {   //when the left mouse button is released
        if (g_bDrawingBox) {
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
            ppcm_width = CC_WIDTH / r.width;
            ppcm_height = CC_LENGTH / r.height;
            DrawRectangle(image, g_rectangle);
            g_bDrawingBox = false;
        }

    }
                        break;

    }
}

int main() {
    //Mat src;
    VideoCapture cap;
    /*
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
        */
    string name =  getenv("USER");
    Mat src = cv::imread("/home/"+name+"/Capstone_Project/catkin_ws/src/vision_package/assets/board.jpg");
    Size fcc(7,7);
    //resize(src, src, Size(), 0.25, 0.25, INTER_CUBIC);
    resize(src, src, Size(1000,1000));
    Point initial_size(src.rows,src.cols); 
    vector<Point2f> corner;
    bool found = findChessboardCorners(src,fcc,corner);
    Size target_size(128,128);
    resize(src,src, target_size);
    Point scale(target_size.width/initial_size.x, target_size.height/initial_size.y);

    std::cout << "Final occupancy generation candidate image: "<<src.rows << " cols: " << src.cols << std::endl;
    imshow("Src", src);
    Mat tempImage,center;

    i = 0;
    /*
    setMouseCallback("Src", mouse_callback, (void*)&src);
    while (1) {
        //cap.read(src);
        src.copyTo(tempImage);
        if (g_bDrawingBox)
            DrawRectangle(tempImage, g_rectangle);
        imshow("Src", tempImage);
        if (waitKey(10) == 27)  // stop drawing rectanglge if the key is 'ESC'
            break;
    }
    */
    
    /*
        bottomm left: 48
        top left: 42
        top right: 0
        bottom right: 6
    */
    src_vertices[0] = corner[48]*0.128;
    src_vertices[1] = corner[42]*0.128;
    src_vertices[2] = corner[0]*0.128;
    src_vertices[3] = corner[6]*0.128;
    float midx= src.cols/2;
    int square_scale=36;
    float midy=src.rows-square_scale;

    //X axis- horizontal; Y axis- vertical
    dst_vertices[1] = Point(midx-square_scale, midy-square_scale);
    dst_vertices[2] = Point(midx+square_scale, midy-square_scale);
    dst_vertices[3] = Point(midx+square_scale, midy+square_scale);
    dst_vertices[0] = Point(midx-square_scale, midy+square_scale);
    ppcm_width = CC_WIDTH /(square_scale*2);
    ppcm_height = CC_LENGTH /(square_scale*2);
    std::cout << "ppcm width: " << ppcm_width << " ppcm_height: " << ppcm_height << std::endl;


    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    
    //cv::Mat_<double> M(3,3);
    //M << 0.4673847726070066, 2.740971070200859, 224.895161512139,
    //-0.137309303776736, 5.090761693240268, 356.2619469772989,
    //-0.0001959789858882858, 0.005527060989712946, 1;
    std::cout << "Perspective Transform Matrix: " << M << std::endl;
    Mat dst(src.rows, src.cols, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
    cout<<"Final homography image: rows "<<dst.rows << "cols" <<dst.cols;
    drawChessboardCorners(src,fcc,Mat(corner),found);
    imshow("src", src);
    namedWindow("dst");
    
    imshow("dst", dst);
    setMouseCallback("dst",CallBackFn,NULL);
    cout<<corner.size()<<endl;
    for(int i=0;i<corner.size();i++) {
        cout<<corner[i]<<endl;
    }
    waitKey();
}

