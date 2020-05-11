#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>//坐标空间使得机器人能够到达
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

namespace my_planning
{
   
    class MyPlanningClass 
    {
        public:
        MyPlanningClass(ros::NodeHandle& node);
        //主要的几个method
        void goToPoseGoal();
        void goToJointState();
        void cartesianPath();


        void resetValues();
        void addObjects();
        void makeBox(std::string blk_name,double* pose);//给虚拟的空间添加虚拟的障碍物
        void removeObject();

        void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client;

        public:
        const std::string PLANNING_GROUP = "magician_arm";
        control_msgs::FollowJointTrajectoryGoal goal;
       

        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface virtual_world;
        const robot_state::JointModelGroup* joint_model_group;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        ros::Subscriber subPointCloud; 

        geometry_msgs::Pose target_pose1;
    };







}