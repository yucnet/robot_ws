#include "sorting/sorting.h"

sorting::sorting( ros::NodeHandle& node_):
    node(node_),it_(node_)
{
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,&sorting::callback, this);//订阅节点，该节点在USB_cam/image_raw中发布。
    //center_point_pub_= nh_.advertise<opencvtest::pixel_point>("pixel_center_axis",1000);//话题名称和接受队列缓存消息条数；
    //cv::namedWindow();
}

void sorting::callback( const sensor_msgs::ImageConstPtr& msg ) 
{
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
   
    this->picture = cv_ptr->image.clone();
    process( &(this->hsv_red) );
  //  process(cv_ptr,yellow);
  //  process(cv_ptr,blue);
  //  process(cv_ptr,green);
  //  process(cv_ptr,purple);
    cv::waitKey(1); 

}

void sorting::process( vector< int >* ptr ) 
{
  cv::Mat drawmap = this->picture; //画布
  cv::Mat clone =picture.clone(); 
  
  cv::cvtColor(clone,clone,CV_BGR2HSV);
  cv::inRange(clone,cv::Scalar( this->hsv_red[1], this->hsv_red[3], this->hsv_red[5] ),
                 cv::Scalar( this->hsv_red[2], this->hsv_red[4], this->hsv_red[6]),clone );
  cv::Mat binary = clone.clone();
  cv::medianBlur(binary,binary,25);
  
  cv::findContours(clone, this->contours, this->hierarcy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  if( this->contours.size() == 0 )
    return;
  cout<< "we detected " << this->contours.size() << "contour(s)" <<endl;
  vector<cv::Rect> rect;
  for(int i = 0;i <this->contours.size(); i++){

    rect.push_back(cv::boundingRect(this->contours[i]));

    if(rect[i].area()>1500){
        cv::rectangle(drawmap, rect[i],cv::Scalar(0,0,255),3);
    }

    cv::Point2f center(0.5*(rect[i].tl().x+rect[i].br().x), 0.5*(rect[i].tl().y+rect[i].br().y));
    this->points.push_back(center);
    //center_point_pub_.publish(msgs);
  }
  cv::imshow("Min Rec",drawmap);
  cv::imshow("bin",binary);
}























