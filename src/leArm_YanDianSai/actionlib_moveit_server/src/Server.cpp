#include <Server/Server.h>
//构造函数的实现
Server::Server():
    as(nh,"learm_controller/follow_joint_trajectory",boost::bind(),false)
    /*第二个参数是action的名字，
    是action服务端与客户端配对的依据，
    在前一篇文章中讲的，controllers.yaml中写的参数name和action_ns
    共同决定了客户端action的名字，因此，服务端的名称必须参照controllers.yaml中的参数来写才能配对。
    */
{


}

//让机械臂动起来的关键环节
void Server::goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
    //复制形参中的goal,修改传递出去,但不破坏原始数据
    server::Goal goal = *gh.getGoal();

    this->goalHandle_ = gh;
    



}