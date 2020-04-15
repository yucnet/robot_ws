#include <server/server.h>
//moveit构造函数的实现
moveitServer::moveitServer(ros::NodeHandle& n):
    as(n,"learm_controller/follow_joint_trajectory",
                boost::bind(&moveitServer::goalCB),false)
{
    as.start();

}

//让机械臂动起来的关键moveit环节
void moveitServer::goalCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, server* as )
{
    //复制形参中的goal,修改传递出去,但不破坏原始数据
}