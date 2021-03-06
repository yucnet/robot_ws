#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <learm_robot_msgs/GoalPoint.h>
#include <std_msgs/Float32MultiArray.h>
 
#include <sstream>
#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
using namespace std;
 
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
 
std::basic_string<char> joint1;
std::basic_string<char> joint2;
std::basic_string<char> joint3;
std::basic_string<char> joint4;
std::basic_string<char> joint5;
std::basic_string<char> joint6;
std::basic_string<char> joint7;
 
/* 目标位置 */
double cp1;
double cp2;
double cp3;
double cp4;
double cp5;
double cp6;
 
/* 目标位置 */
double tp1;
double tp2;
double tp3;
double tp4;
double tp5;
double tp6;
ros::Publisher* pubptr = NULL;
/* 收到action的goal后调用的回调函数 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
{
    ROS_INFO_STREAM("begin to exec!");
    learm_robot_msgs::GoalPoint msg;
	//ros::Rate rate(1);
  
    joint1 = goal->trajectory.joint_names[0];
    joint2 = goal->trajectory.joint_names[1];
    joint3 = goal->trajectory.joint_names[2];
    joint4 = goal->trajectory.joint_names[3];
    joint5 = goal->trajectory.joint_names[4];
   
    //第一个waypoint的关节角度
    cp1 = goal->trajectory.points[0].positions[0];
    cp2 = goal->trajectory.points[0].positions[1];
    cp3 = goal->trajectory.points[0].positions[2];
    cp4 = goal->trajectory.points[0].positions[3];
    cp5 = goal->trajectory.points[0].positions[4];

    msg.joint1 =  goal->trajectory.points[0].positions[0];
    msg.joint2 =  goal->trajectory.points[0].positions[1];
    msg.joint3 =  goal->trajectory.points[0].positions[2];
    msg.joint4 =  goal->trajectory.points[0].positions[3];
    msg.joint5 =  goal->trajectory.points[0].positions[4];
  
    // //第二个waypoint的关节角度
    // tp1 = goal->trajectory.points[1].positions[0];
    // tp2 = goal->trajectory.points[1].positions[1];
    // tp3 = goal->trajectory.points[1].positions[2];
    // tp4 = goal->trajectory.points[1].positions[3];
    // tp5 = goal->trajectory.points[1].positions[4];
   
 
	// 并且按照1hz的频率发布进度feedback
	// control_msgs::FollowJointTrajectoryFeedback feedback;
    //feedback = NULL;
    //as->publishFeedback(feedback);
 
    // 当action完成后，向客户端返回结果
    //printf("goal=[%f,%f,%f,%f,%f,%f]\n",tp1,tp2,tp3,tp4,tp5,tp6);
    while (1)
    {
       
        pubptr->publish(msg);
        cout<<"发送成功!"<<endl;
    }
    
    as->setSucceeded();
    ROS_INFO_STREAM("exec end!");
}
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_server");
	ros::NodeHandle nh;
    cout<<"cheuhcusc"<<endl;
    ros::Publisher pub= nh.advertise<learm_robot_msgs::GoalPoint>("learm_joint/send_arm_data",100);
    pubptr = &pub;
	Server server(nh, "learm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
    cout<<"cheuhcusc"<<endl;
	server.start();
    cout<<"cheuhcusc"<<endl;
    ROS_INFO_STREAM("waiting for a goal!"<<"　"<<" ...");
	ros::spin();
}
