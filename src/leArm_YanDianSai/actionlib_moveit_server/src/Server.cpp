#include <Server/Server.h>
//构造函数的实现
Server::Server():
    as(nh,"learm_controller/follow_joint_trajectory",
                boost::bind(&Server::goalCB),false)
{


}

//让机械臂动起来的关键环节
void Server::goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
    //复制形参中的goal,修改传递出去,但不破坏原始数据
    server::Goal goal = *gh.getGoal();

    this->goalHandle_ = gh;
    



}