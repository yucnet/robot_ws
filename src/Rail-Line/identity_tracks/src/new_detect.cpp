#include<ros/ros.h>
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
class detect{
public:
    detect(){




    }
    cv::Mat frame;
    cv::Mat grayFrame;
    cv::Mat edgeFrame;
    cv::Mat equalizeHistGrayFrame;
    cv::Mat medianBlurFrame;
    cv::Mat cannyFrame;
    cv::Mat vertical;
    cv::Mat horizontal;
    cv::Mat horizontalStructrue;
    cv::Mat   verticalStructrue;
    cv::Mat binary;

    vector<Vec4i> lines;

    int horizontalSize; 
    int verticlSize;
    void preProcess();
    void lineDetection();
    Mat setROI(const Mat &imgBin);

};

int main(int argc,char** argv)
{
    
    ros::init(argc,argv,"test");
    ros::NodeHandle n;
    detect de;
    cv::Mat& frame = de.frame;
    //cv::namedWindow("original",cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("ehgray",cv::WINDOW_AUTOSIZE);
    string path = "/home/zhangeaky/robot_ws/src/Rail-Line/identity_tracks/Rail.mp4";
    cv::VideoCapture cap(path);
    while (ros::ok())
    {
        cap >> de.frame;
        cv::resize(frame,frame,cv::Size(frame.cols/2,frame.rows/2)
                        ,0,0,cv::INTER_LINEAR);
        de.preProcess();
        // cv::imshow("horizontal",de.horizontal); 
        // cv::imshow("vertical",de.vertical); 
       
        
        cv::waitKey(300);
    }
    
    return 0;
}

void detect::preProcess()
{
    cv::cvtColor(this->frame,this->grayFrame,COLOR_BGR2GRAY);

    int gray_mean = cv::mean(this->grayFrame)[0];

    int max_canny_val;
  //对上界选值
    if (gray_mean > 120){
        max_canny_val = 100; //for rails covered in snow 被雪覆盖的场景
    }else{
        max_canny_val =  0.003594*gray_mean*gray_mean*gray_mean 
                    - 0.8398*gray_mean*gray_mean + 64.05*gray_mean - 1460;
    }
    
    if (max_canny_val > 250){
        max_canny_val = 250;
    }
    if (max_canny_val < 100){
        max_canny_val = 100;
    }
    cout<<max_canny_val<<endl;
      
    cv::equalizeHist(this->grayFrame,this->equalizeHistGrayFrame);
    cv::medianBlur(this->equalizeHistGrayFrame,this->medianBlurFrame,3);
    //cv::GaussianBlur(this->equalizeHistGrayFrame,this->medianBlurFrame,cv::Size(3,3),1);
    cv::Sobel(this->medianBlurFrame,this->cannyFrame,this->medianBlurFrame.depth(),1,0);
    cv::imshow("before",this->cannyFrame); 
    cv::convertScaleAbs(this->cannyFrame,this->cannyFrame);
    //cv::Canny(this->medianBlurFrame,this->cannyFrame,200,max_canny_val);
    cv::imshow("after",this->cannyFrame); 
    
    this->horizontalSize = this->cannyFrame.cols/150;
    cout<<"h:"<<this->horizontalSize<<endl;
    this->horizontalStructrue = cv::getStructuringElement(MORPH_RECT, Size(horizontalSize,1));
    erode(this->cannyFrame, this->horizontal, this->horizontalStructrue, Point(-1, -1));
    dilate(this->horizontal, this->horizontal, this->horizontalStructrue, Point(-1, -1));
    
    this->verticlSize = this->cannyFrame.rows/10;
    cout<<"v:"<<this->verticlSize<<endl;
    erode(this->cannyFrame, this->vertical, this->verticalStructrue, Point(-1, -1));
    dilate(this->vertical, this->vertical, this->verticalStructrue, Point(-1, -1));

    cv::Mat temp = this->cannyFrame - this->vertical- this->horizontal;
    //cv::imshow("sobel2",temp);
    //cv::Mat temp;
}

void detect::lineDetection()
{
    
   // cv::HoughLinesP(this->binary,this->lines,);



}
Mat detect::setROI(const Mat &imgBin) // making ROI polygon and intersecting with canny
{
  Mat imgPoly(imgBin.size(), imgBin.type());
  Mat Result;
  int lineType = LINE_8;
  const Point* ppt[1] = {poly_points[0]};
  int npt[] = {4};

  imgPoly.setTo(0);//所有像素设为零
  fillPoly(imgPoly, ppt,npt,1, Scalar(255, 255, 255), lineType);
  bitwise_and(imgPoly, imgBin, Result);
  return Result;
}