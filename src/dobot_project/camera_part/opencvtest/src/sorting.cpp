/*********************************ROS***********************************************/
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencvtest/pointQueue.h>
#include <opencvtest/pointQueueRequest.h>
#include <real_sense/GetDistance.h>
/**********************************OPENCVLIBRARIES**********************************/
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>//新式C++风格图像处理函数
#include <opencv2/highgui/highgui.hpp>//C++风格的显示、滑动条鼠标及输入输出相关

/**********************************OPENCVLIBRARIES**********************************/
#include <iostream>
#include <boost/thread.hpp>
#include <dobot/GetPose.h>
using namespace std;
/**********************************REALSENSE*****************************************/
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_context.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
/**********************************HSV***********************************************/
vector<int> hsvred    ={0, 0 ,10, 160,255, 130,255};
vector<int> hsvyellow ={0, 26,34,   43,255, 46,255};
vector<int> hsvblue   ={0, 100,124, 43,255, 46,255};
vector<int> hsvgreen  ={0, 45,77,   43,255, 46,255};
vector<int> hsvpurple ={0, 125,155, 43,255, 46,255};//H:125-155

class realsense
{
public:
rs2::pipeline pipline;
rs2::frameset frames;
rs2::frame frame;
rs2::context ctx;
rs2::config cfg;
rs2::device dev;

public:
realsense()
{
  cout<<"realsense构造函数"<<endl;
  auto list = this->ctx.query_devices();		
		if(list.size()==0)
			{
				throw runtime_error("No Device Detected !");
			}
			else
			{
				rs2_camera_info info;
				this->dev = list.front();//返回第一个设备.
				auto information = dev.get_info(info);
				cout<<"设备信息: "<<information<<endl;
				auto sensors = dev.query_sensors();
				cout<<"传感器数目:"<<sensors.size()<<endl;
				rs2::sensor sensor1 = sensors[0];
				rs2::sensor sensor2 = sensors[1];
				
				auto info_sensor_1 = sensor1.get_info(info);
				auto info_sensor_2 = sensor2.get_info(info);
				cout<<"传感器1:"<<info_sensor_1<<endl;
				cout<<"传感器2:"<<info_sensor_2<<endl;
			}
      //cfg.enable_all_streams();
      cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
      // cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_DISTANCE,30);
      //this->pipline.start(this->cfg);
      this->pipline.start(cfg);
}
};

class cube
{
public:
  cube(const string name,vector<int> hsv)
  {
    this->name = name;
    this->HSV  = hsv;
  }
  std::string getName()
  {
    return this->name;
  }
  void clearPointsUV()
  {
    this->points_uv.clear();
  }
  void setpointsUV(cv::Point2i& point)
  {
    this->points_uv.push_back(point);
  }
  vector<int>& getHSV()
  {
    return this->HSV;
  }
  vector< vector<cv::Point> >&  getContours()
  {
    return this->contours;
  }
  void  setCubeNumbers(int number)
  {
    this->cube_num = number;
  }
  int getCubeNumbers()
  {
    return this->cube_num;
  }
  vector<cv::Point2i>  getPoints()
  {
    return this->points_uv;
  }
  vector<double> distance_queue;
private:
  string name;
  vector< vector<cv::Point> >  contours;
  vector<cv::Point2i>  points_uv;//存放同一种颜色的像素坐标的点队列
  
  vector<int> HSV;//HSV色域表
  int cube_num;
};
/*
订阅深度图像的话题,订阅彩色图像的话题,开辟两个线程,分别进行处理.
注册一个Service用于存储识别的物体的像素信息
 */

class sorting
{
private:
ros::NodeHandle n;
image_transport::ImageTransport it;
image_transport::Subscriber* pointer_image_sub = new image_transport::Subscriber;
image_transport::Subscriber* depth_frame_sub = new image_transport::Subscriber;


ros::ServiceServer server; 
ros::ServiceClient client;



int timer = 0;  
bool ifsaved = false;
bool ifdepthsaved = false;
bool finished = false;
float distance; 


public:

cv::Mat camera_captured;
cv::Mat depth_image;

sorting()
: it(n)
{
  *(this->pointer_image_sub) = it.subscribe("/camera/color/image_raw",10,&sorting::callbackImage,this);
  *(this->depth_frame_sub) = it.subscribe("/camera/aligned_depth_to_color/image_raw",10,&sorting::callbackImageDepth,this);
  this->server = n.advertiseService("pixel_Axis",&sorting::callback,this);
  this->client = n.serviceClient<dobot::GetPose>("/dobot/GetPose");
}
~sorting()
{
  cv::destroyAllWindows();
}

bool callback(opencvtest::pointQueueRequest &req, opencvtest::pointQueueResponse &res);

void callbackImage(const sensor_msgs::ImageConstPtr& msg);
void callbackImageDepth(const sensor_msgs::ImageConstPtr& msg);

void colorDistinguish(cv::Mat& src, cube& cube);
void loadPicture();
void getPicture();
void getDepthPicture();
void takepoint(int u, int v);
};

cube red("red",hsvred);
cube yellow("yellow",hsvyellow);
cube blue("blue",hsvblue);
cube green("green",hsvgreen);
cube purple("purple",hsvpurple);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_distinguish");
  sorting ic;
  //ic.loadPicture();
  //ic.getDepthPicture();
  //ic.getPicture();
  //spinner.start();
  //ros::waitForShutdown();
  ros::spin();
  return 0;
}


void sorting::takepoint(int u, int v)
{
  cout<<endl<<endl<<"*********"<<endl;
  for(int i=-1;i<2;i++)
    for(int j=-1;j<2;j++)
      {
        cout<<this->depth_image.at<unsigned short>(u+i,v+j)/1000.00<<endl;
      }
  cout<<"*********"<<endl<<endl;
}

void sorting::callbackImage(const sensor_msgs::ImageConstPtr& msg) 
{
 
  // dobot::GetPose srv;
  // this->client.call(srv);
  // cv::namedWindow("sorting");
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  this->camera_captured = cv_ptr->image;
  cv::imshow("sorting",cv_ptr->image);
  int c = cv::waitKey(1);

  if(c==' ')
    {
      ROS_INFO("PICTURE HAS BEEN SAVED!");
      this->ifsaved = cv::imwrite("/home/zhangeaky/图片/picForROS/color.jpg",cv_ptr->image);
      delete this->pointer_image_sub;
      cv::destroyWindow("sorting");
      this->loadPicture();
      cout<<"finished: "<<this->finished<<endl;
    }
}

void sorting::callbackImageDepth(const sensor_msgs::ImageConstPtr& msg) 
{
  cout<<"!!!!"<<endl;
  if(!this->finished)
    {
      return;
    }
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
  this->depth_image = cv_ptr->image;

  takepoint(278,419);
  takepoint(282,292);
  takepoint(406,295);
  takepoint(415,423);
  takepoint(350,354);



  cv::imwrite("/home/zhangeaky/图片/picForROS/depth_align_to_color.jpg",cv_ptr->image);
  if(green.getCubeNumbers()==0&&red.getCubeNumbers()==0)
    {
      cout<<"没有检测到任何目标物块."<<endl;
      // cout<<"ijn"<<endl;
      // cout<<this->depth_image.at<unsigned short>(352,396)/1000.00<<endl;
      // cout<<this->depth_image.at<unsigned short>(296,325)/1000.00<<endl;
      // cout<<this->depth_image.at<unsigned short>(290,231)/1000.00<<endl;
      // cout<<this->depth_image.at<unsigned short>(388,226)/1000.00<<endl;
      // cout<<this->depth_image.at<unsigned short>(344,275)/1000.00<<endl;
    }
  else
    {
       for(int i=0;i<green.getCubeNumbers();i++)
        {
          double d = this->depth_image.at<unsigned short>(green.getPoints()[i].x+640*green.getPoints()[i].y)/1000.00;
          //cout<<d<<endl;
          green.distance_queue.push_back(d);
        }

      for(int i=0;i<red.getCubeNumbers();i++)
        {
          double d = this->depth_image.at<unsigned short>(red.getPoints()[i].x+640*red.getPoints()[i].y)/1000.00;
          //cout<<d<<endl;
          red.distance_queue.push_back(d);
        }
    }
  
  //delete this->depth_frame_sub;
}

void sorting::loadPicture()
{

  ROS_INFO("start to process picture");
  this->camera_captured = cv::imread("/home/zhangeaky/图片/picForROS/color.jpg");
  cv::Mat src = this->camera_captured;
  cout<<endl<<"%%%%%%%%%%%%%%%%%%%%"<<endl;
  this->colorDistinguish(src,green);
  this->colorDistinguish(src,red);
  // this->colorDistinguish(src,purple);
  // this->colorDistinguish(src,yellow); 
  cout<<"%%%%%%%%%%%%%%%%%%%%"<<endl;   
  this->finished =true;
 }

void sorting::colorDistinguish(cv::Mat& src, cube& cube)
{ 
  ROS_INFO("DETECTING!");
  cv::Mat drawmap = src.clone(); 
  cv::Mat clone = src.clone(); 
  cv::Mat binary;
  vector<cv::Mat> hue;

  vector<cv::Vec4i> hierarcy;

  cv::cvtColor(clone,clone,CV_BGR2HSV);
  cv::split(clone,hue);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
  cv::dilate(hue[0],hue[0],kernel);
  cv::equalizeHist(hue[2],hue[2]);
 
  cv::Mat newsrc;
  cv::merge(hue,newsrc);

  cv::inRange(clone, cv::Scalar(cube.getHSV()[1],cube.getHSV()[3],cube.getHSV()[5]),cv::Scalar(cube.getHSV()[2],cube.getHSV()[4],cube.getHSV()[6]),clone);
  binary = clone.clone();
  cv::medianBlur(binary,binary,25);
  cv::findContours(binary, cube.getContours(), hierarcy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  if(cube.getContours().size()==0)
  {
    cout<<"没有检测到"<<cube.getName()<<endl;
    return;
  }

  cout<<"检测到:"<<cube.getContours().size()<<"个"<<cube.getName()<<"物块"<<endl;
  cube.setCubeNumbers(cube.getContours().size());

  vector<cv::Rect> rects;
  cv::Rect rect;

  cv::Point2i point;//像素中心点
  cv::RotatedRect temp;
  cv::Point2f vertices[4];
  cube.clearPointsUV();
  for(int i = 0;i < cube.getContours().size(); i++)
      {
        temp = cv::minAreaRect( cube.getContours()[i] );
        temp.points(vertices);
        rect={*(vertices),*(vertices+2)};
        rects.push_back(rect);
        cout<<"面积:"<<rects[i].area()<<endl;
        
        if(rects[i].area()>0)
             {
                for (int j = 0; j <= 3; j++)
                    {
                      cv::line(this->camera_captured,vertices[j], vertices[(j + 1) % 4], cv::Scalar(255,255,255),1);
                    }
                point = {static_cast<int>(0.5*(rects[i].tl().x+rects[i].br().x)), static_cast<int>(0.5*(rects[i].tl().y+rects[i].br().y))};
                cout<<"pixel:"<<endl;
                cout<<"("<<static_cast<int>(point.x)<<","<<static_cast<int>(point.y)<<")"<<endl;
                //this->distance = this->depth_image.at<unsigned char>(static_cast<int>(point.x)+640*static_cast<int>(point.y))/1000.00;
                //cube.distance_queue.push_back(this->distance);
                //cout<<distance<<endl;
                cv::Point2i p(static_cast<int>(point.x)+50,static_cast<int>(point.y)+50);
                std::ostringstream stream;
                cv::putText(this->camera_captured, "depth"+to_string(this->distance),p,CV_FONT_HERSHEY_COMPLEX,0.6,(0,0,255));
                cv::imwrite("/home/zhangeaky/图片/picForROS/sample.jpg",this->camera_captured);
                cube.setpointsUV(point);
             }
       
      }

  // cv::imshow("image",this->camera_captured);
  // int key = cv::waitKey(0);
  // if(key==27)
  // {
  //   cv::imwrite("/home/zhangeaky/sorting.jpg",this->camera_captured);
  //   cv::destroyAllWindows();
  //   return;
  // }
}

bool sorting::callback(opencvtest::pointQueueRequest &req, opencvtest::pointQueueResponse &res)
{

  cout<<"caling!!!!!!!"<<endl;
  if(green.getCubeNumbers()!=0)
    {
      for(int i=0;i<green.getCubeNumbers();i++)
        {
          res.green.push_back(green.getPoints()[i].x);
          res.green.push_back(green.getPoints()[i].y);
          res.green.push_back(green.distance_queue[i]);
        }
    }
  if(red.getCubeNumbers()!=0)
    {
      for(int i=0;i<red.getCubeNumbers();i++)
      {
        res.red.push_back(red.getPoints()[i].x);
        res.red.push_back(red.getPoints()[i].y);
        res.red.push_back(red.distance_queue[i]);
      }
    }
    return true;
}
