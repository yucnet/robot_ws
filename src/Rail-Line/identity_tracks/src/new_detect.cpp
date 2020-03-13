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
    cv::Mat sobelFrame;
    cv::Mat vertical;
    cv::Mat horizontal;
    cv::Mat horizontalStructrue;
    cv::Mat   verticalStructrue;

    int horizontalSize; 
    int verticlSize;
    void preProcess();
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
        cv::resize(frame,frame,cv::Size(frame.cols/4,frame.rows/4)
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
    cv::equalizeHist(this->grayFrame,this->equalizeHistGrayFrame);
    cv::medianBlur(this->equalizeHistGrayFrame,this->medianBlurFrame,3);
    //cv::GaussianBlur(this->equalizeHistGrayFrame,this->medianBlurFrame,cv::Size(3,3),1);
    cv::Sobel(this->medianBlurFrame,this->sobelFrame,this->medianBlurFrame.depth(),1,0);
    cv::imshow("sobel",this->sobelFrame);  
    this->horizontalSize = this->sobelFrame.cols/150;
    cout<<"h:"<<this->horizontalSize<<endl;
    this->horizontalStructrue = cv::getStructuringElement(MORPH_RECT, Size(horizontalSize,1));
    erode(this->sobelFrame, this->horizontal, this->horizontalStructrue, Point(-1, -1));
    dilate(this->horizontal, this->horizontal, this->horizontalStructrue, Point(-1, -1));
    
    this->verticlSize = this->sobelFrame.rows/10;
    cout<<"v:"<<this->verticlSize<<endl;
    erode(this->sobelFrame, this->vertical, this->verticalStructrue, Point(-1, -1));
    dilate(this->vertical, this->vertical, this->verticalStructrue, Point(-1, -1));

    cv::Mat temp = this->sobelFrame - this->vertical;
    cv::imshow("sobel2",temp);
    


}