#include <client/client.h>
using namespace std;
moveitClient::moveitClient(ros::NodeHandle& node):
    node(node),
    moveit_client(node,"learm_controller/follow_joint_trajectory"),
    group("test1")
{
    ROS_INFO("Movegroup test1 init successfully");
    // group.setGoalPositionTolerance(0.01);//单位米
    // group.setGoalOrientationTolerance(0.05);//单位弧度
    ROS_INFO("wait for server");
    this->moveit_client.waitForServer();
    ROS_INFO("action server started, prepare to sending goal");
    this->goToPoseGoal();
    //this->CartesianPath(0.08,0.08,0.15);
    while(1);
  
    //goal.trajectory.joint_names = {"joint1","joint2"," joint3"," joint4","joint5"};
    goal.trajectory.joint_names = this->myPlan.trajectory_.joint_trajectory.joint_names;
    goal.trajectory.points = this->myPlan.trajectory_.joint_trajectory.points;
     
    // goal.trajectory.points[trajectory_index_].velocities.resize(5);
    // goal.trajectory.points[trajectory_index_].accelerations.resize(5);
    // for (size_t i = 0; i < 5; ++i)
    // {
    //     goal.trajectory.points[trajectory_index_].velocities[i] = 0.0;
    //     goal.trajectory.points[trajectory_index_].accelerations[i] = 0.0;
    // }

    goal.trajectory.points[trajectory_index_].time_from_start = ros::Duration(5.0);
    ROS_INFO("First trajectory prepare ok!");
    
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    this->moveit_client.sendGoal(goal); 
    ROS_INFO_STREAM("Send joint trajectory goal to server successfully!");
    // goal.trajectory.header.frame_id = "base_link";
    // goal.trajectory.header.stamp = ros::Time::now();
}

void moveitClient::CartesianPath(double x, double y, double z)
{
    cout<<"start cartesian planning!"<<endl;
    moveit_msgs::RobotState startPose;
    startPose.joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
    startPose.joint_state.position = {0.00,0.00,0.00,0.00,0.00};
    startPose.joint_state.header.stamp = ros::Time::now();
    startPose.joint_state.header.frame_id = "base_link";
    group.setPoseReferenceFrame("base_link");
    //sssssscout<<"debug"<<endl;
    //group.setStartState(startPose);
    cout<<"the start pose has been settled"<<endl;
    cout<<"末端: "<<group.getEndEffectorLink()<<endl;
    //设置目标位置所在参考坐标系
    group.allowReplanning(true);//允许重新规划
    group.setNumPlanningAttempts(10);

    group.setGoalPositionTolerance(0.001);
    group.setGoalOrientationTolerance(0.01);

    geometry_msgs::Pose point;
    tf::Quaternion q;
    q.setRPY(0,0,0);
    point.orientation.w = q.getW();
    point.orientation.x = q.getX();
    point.orientation.y = q.getY();
    point.orientation.z = q.getZ();
    point.position.x = x;
    point.position.y = y;
    point.position.z = z;
    std::vector<geometry_msgs::Pose> wayPoints;
    wayPoints.push_back(point);
    // point.position.z += 0.2;
    // point.position.x += 0.2;
    // point.position.y -= 0.2;
    // wayPoints.push_back(point);
    this->group.setMaxVelocityScalingFactor(0.1);
    //moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    cout<<"begin to plan the trajectory"<<endl;
    double fraction = group.computeCartesianPath(wayPoints,eef_step,jump_threshold,trajectory);
    cout<<"fraction: "<<fraction<<endl;
    cout<<"产生轨迹点个数: "<<trajectory.joint_trajectory.points.size()<<endl;
    cout<<"坐标: "<<trajectory.joint_trajectory.header.frame_id<<endl;
    for(int i=0;i<trajectory.joint_trajectory.points.size();i++)
        {
            cout<<endl<<"----------"<<endl;
            cout<<"关节名称: "<<trajectory.joint_trajectory.joint_names[i]<<endl;
            for(int j=0;j<trajectory.joint_trajectory.points[i].positions.size();j++)
            {
                cout<<trajectory.joint_trajectory.points[i].positions[j]<<endl;
            }
            cout<<"----------"<<endl;
        }
    //group.move();
    ROS_INFO_STREAM("Plan has finished!");
}

void moveitClient::goToPoseGoal()
{
    cout<<"start"<<endl;
    moveit_msgs::RobotState startPose;
    startPose.joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
    startPose.joint_state.position = {0.00,0.00,0.00,0.00,0.00};
    startPose.joint_state.header.stamp = ros::Time::now();
    startPose.joint_state.header.frame_id = "base_link";
    group.setPoseReferenceFrame("base_link");
    //sssssscout<<"debug"<<endl;
    //group.setStartState(startPose);
    cout<<"the start pose has been settled"<<endl;
    group.setNamedTarget("Home");
    group.allowReplanning(true);//允许重新规划
    group.setNumPlanningAttempts(10);
    //group.getCurrentState();
    //group.setPositionTarget(0.18,-0.2,0.05,"link5");
    //group.get
    //group.setPoseTarget(this->target_pose1,"magician_end_link");
    //group.plan(this->myPlan);
    bool success  = (group.plan(this->myPlan) ==
                     moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout<<"success: "<<success<<endl;
    cout<<"size:"<<this->myPlan.trajectory_.joint_trajectory.points.size()<<endl;

    // if(!success)
    //     throw std::runtime_error("no planning found");
    // //blocking
    cout<<"to move"<<endl;
    //group.move();
}

void moveitClient::goToJointState()
{   
    robot_state::RobotState current_state = *group.getCurrentState();
    std::vector<double> joint_positions;
    //joint_model_group = current_state.getJointModelGroup("test1");
    //current_state.copyJointGroupPositions(joint_model_group, joint_positions);

    joint_positions[0] = -1.0;

    group.setJointValueTarget(joint_positions);
    bool success  = (group.plan(this->myPlan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout<<"size:"<<this->myPlan.trajectory_.joint_trajectory.points.size()<<endl;
    if(!success)
        throw std::runtime_error("no planning found");
    //blocking
    group.move();

}


actionlib::SimpleClientGoalState moveitClient::getTrajectoryState()
{
    return moveit_client.getState();
}
moveitClient::~moveitClient()
{

}
void moveitClient::doneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
    ROS_INFO_STREAM("Task Finished!");

}

void moveitClient::activeCb()
{
    ROS_INFO("Goal just went active!");
}

