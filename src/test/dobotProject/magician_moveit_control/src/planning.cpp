#include <planning/planning.h>

using namespace std;
namespace my_planning
{
MyPlanningClass::MyPlanningClass(ros::NodeHandle& node):
move_group(this->PLANNING_GROUP),
client(node,"magician_arm_controller/follow_joint_trajectory")
{
    this->subPointCloud = node.subscribe("/camera/depth/color/points",100,&MyPlanningClass::callback,this);
    tf::Quaternion q;
    q.setRPY(0,0,0);
    this->target_pose1.orientation.w = q.getW();
    this->target_pose1.orientation.x = q.getX();
    this->target_pose1.orientation.y = q.getY();
    this->target_pose1.orientation.z = q.getZ();
    this->target_pose1.position.x = 0.00;
    this->target_pose1.position.y = 0.15;
    this->target_pose1.position.z = 0.0;
    
    this->move_group.allowReplanning(true);
    this->move_group.setNumPlanningAttempts(10);
}

//规划指定的目标点
void MyPlanningClass::goToPoseGoal()
{
    cout<<"start "<<endl;
    ROS_INFO("Start gotoPoseGoal funtion!");
    move_group.getCurrentState();
    move_group.setStartStateToCurrentState(); 
    // move_group.setPoseReferenceFrame("world");
    move_group.setNamedTarget("home");
    // //move_group.setPositionTarget(0.38,-0.2,0.05,"magician_end_link");
    // move_group.setPoseTarget(this->target_pose1,"magician_end_link");
    bool success  = (move_group.plan(this->my_plan) ==
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout<<"size:"<<this->my_plan.trajectory_.joint_trajectory.points.size()<<endl;
    cout<<"position:"<<this->my_plan.trajectory_.joint_trajectory.points[0].positions.size()<<endl;
    this->goal.trajectory.header.stamp = ros::Time::now();
    this->goal.trajectory.joint_names = this->my_plan.trajectory_.joint_trajectory.joint_names;
    this->goal.trajectory.points = this->my_plan.trajectory_.joint_trajectory.points;
    client.sendGoal(this->goal);
    // if(!success)
    //     throw std::runtime_error("no planning found");
    // move_group.move();
    ROS_INFO("Goal have been sent!"); 
}


void MyPlanningClass::goToJointState()
{   
    robot_state::RobotState current_state = *move_group.getCurrentState();
    std::vector<double> joint_positions;
    joint_model_group = current_state.getJointModelGroup(this->PLANNING_GROUP);
    current_state.copyJointGroupPositions(joint_model_group, joint_positions);

    joint_positions[0] = -1.0;

    move_group.setJointValueTarget(joint_positions);
    bool success  = (move_group.plan(this->my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout<<"size:"<<this->my_plan.trajectory_.joint_trajectory.points.size()<<endl;
    
    if(!success)
        throw std::runtime_error("no planning found");
    //blocking
    move_group.move();

}

void MyPlanningClass::cartesianPath()
{
    std::vector<geometry_msgs::Pose> wayPoints;
    wayPoints.push_back(target_pose1);

   
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    cout<<"wait"<<endl;
    double fraction = move_group.computeCartesianPath(wayPoints,eef_step,jump_threshold,trajectory);
    cout<<"fraction: "<<fraction<<endl;
    cout<<trajectory.joint_trajectory.points.size()<<endl;
    cout<<trajectory.joint_trajectory.joint_names[0]<<endl;
    cout<<trajectory.joint_trajectory.header.frame_id<<endl;
    cout<<trajectory.joint_trajectory.points[0].positions[0]<<endl;
    cout<<trajectory.joint_trajectory.points[0].positions[1]<<endl;
    cout<<trajectory.joint_trajectory.points[0].positions[2]<<endl;
    move_group.move();
    ROS_INFO_STREAM("PERCE");
}
    void  MyPlanningClass::resetValues()
    {
        move_group.setStartStateToCurrentState();
        move_group.setMaxAccelerationScalingFactor(1.0);
    }

    void MyPlanningClass::makeBox(std::string blk_name, double* pose)
    {
        moveit_msgs::CollisionObject box;
        box.header.frame_id = move_group.getPlanningFrame();
        cout<<box.header.frame_id<<endl;
        box.id = blk_name;
       
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1;
        primitive.dimensions[1] = 0.1;
        primitive.dimensions[2] = 0.4;
         
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = pose[0];
        box_pose.position.y = pose[1];
        box_pose.position.z = pose[2];
      
        box.primitives.push_back(primitive);
        box.primitive_poses.push_back(box_pose);
        box.operation = box.ADD;

        std::vector<moveit_msgs::CollisionObject> collisionObjects;
        collisionObjects.push_back(box);
        //若不睡眠就不会出现
        ros::Duration(2).sleep();
        virtual_world.addCollisionObjects(collisionObjects);
    }
    void MyPlanningClass::addObjects()
    {         
        double box_pose1[3] = {0.00, 0.20, 0.10};
        makeBox("block_1",box_pose1);
        
        double box_pose2[3] = {0.00, -0.20, 0.20};
        makeBox("block_2",box_pose2);

        double box_pose3[3] = {0.20, 0.20, 0.20};
        makeBox("block_3",box_pose3);

    }
    void MyPlanningClass::removeObject()
    {
        std::vector<std::string> object_ids;
        object_ids.push_back("block_1");
        object_ids.push_back("block_2");

        virtual_world.removeCollisionObjects(object_ids);
    }

    void MyPlanningClass::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        cout<<"接收到点云信息!"<<endl;
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.1, 0.1, 0.1));
        tf::Quaternion q;
        q.setRPY(1.57,1.57,0.00);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),
                                    "magician_base","camera_depth_optical_frame"));
    }
}
   
        
