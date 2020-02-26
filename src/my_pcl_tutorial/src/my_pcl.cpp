#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
 
#include <boost/thread.hpp>
 
/**
 * This tutorial demonstrates the use of custom separate callback queues that can be processed
 * independently, whether in different threads or just at different times.
 */
 
void chatterCallbackMainQueue(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("I heard: [ " << msg->data << "] in thread [" << boost::this_thread::get_id() << "] (Main thread)");
}
 
 
void chatterCallbackCustomQueue(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("I heard: [ " << msg->data << "] in thread [" << boost::this_thread::get_id() << "]");
}
 
/**
 * The custom queue used for one of the subscription callbacks
 */
ros::CallbackQueue g_queue;
void callbackThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
 
  ros::NodeHandle n;
  while (n.ok())
  {
    g_queue.callAvailable(ros::WallDuration(0.01));
  }
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_with_custom_callback_processing");
  ros::NodeHandle n;
 
  /**
   * The SubscribeOptions structure lets you specify a custom queue to use for a specific subscription.
   * You can also set a default queue on a NodeHandle using the NodeHandle::setCallbackQueue() function.
   *
   * AdvertiseOptions and AdvertiseServiceOptions offer similar functionality.
   */
  ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::String>("chatter", 1000,
                                                                              chatterCallbackCustomQueue,
                                                                              ros::VoidPtr(), &g_queue);
  ros::Subscriber sub = n.subscribe(ops);
 
  ros::Subscriber sub2 = n.subscribe("chatter", 1000, chatterCallbackMainQueue);
 
  boost::thread chatter_thread(callbackThread);
 
  ROS_INFO_STREAM("Main thread id=" << boost::this_thread::get_id());
 
  ros::Rate r(1);
  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
 
  chatter_thread.join();
 
  return 0;
}