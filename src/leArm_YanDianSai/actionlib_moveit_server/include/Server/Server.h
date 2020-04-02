#include <ros/ros.h>
#include <actionlib/server/action_server.h>//action服务端的相关定义
//action服务端的目标控制句柄定义，与接收的目标相关联后，可以用来实现action的信息反馈等操作
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

/*注意control_msgs::FollowJointTrajectoryAction这个类型，
    这是一个ROS自带的针对机械臂关节角的action类型，
    moveit作为action客户端也是这个类型，因此，这里编写的服务端必须保持类型一致。
*/
typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> server;
typedef actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goalHandle;
class Server
{
public:
    //定义服务端对象
    server as();
    //服务端goal控制句,处理和转换来自客户端的goal数据
    goalHandle goalHandle_;

    //反馈执行情况
    control_msgs::FollowJointTrajectoryResult result;
    ros::NodeHandle nh;

public:
    Server();
    virtual ~Server();

    void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);

};