#include <planning/planning.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
namespace my_planning
{
    //规划指定的目标点
    void MyPlanningClass::goToPoseGoal()
    {
        move_group.setPoseTarget(this->target_pose1);
        bool success  = (move_group.plan(this->my_plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

        //my_plan.trajectory_
        if(!success)
            throw std::runtime_error("no planning found");
        //blocking
        move_group.move();
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
        
        if(!success)
            throw std::runtime_error("no planning found");
        //blocking
        move_group.move();

    }

    void MyPlanningClass::cartesianPath()
    {
        std::vector<geometry_msgs::Pose> wayPoints;
        wayPoints.push_back(target_pose1);

        geometry_msgs::Pose target_pose2 = target_pose2;
        target_pose2.position.z -= 0.2;
        wayPoints.push_back(target_pose2);

        target_pose2.position.z -= 0.2;
        wayPoints.push_back(target_pose2);

        target_pose2.position.z += 0.2;
        target_pose2.position.x += 0.2;
        target_pose2.position.y -= 0.2;
        wayPoints.push_back(target_pose2);

        move_group.setMaxVelocityScalingFactor(0.1);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(wayPoints,eef_step,jump_threshold,trajectory);
    
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
        box.id = blk_name;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions[0] = 0.2;
        primitive.dimensions[0] = 0.2;
        primitive.dimensions[0] = 1.2;

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
        double box_pose1[3] = {0.0, 0.77, 0.0};
        makeBox("block_1",box_pose1);

        double box_pose2[3] = {0.6, -0.77, 0.0};
        makeBox("block_2",box_pose2);

    }

    void MyPlanningClass::removeObject()
    {
        std::vector<std::string> object_ids;
        object_ids.push_back("block_1");
        object_ids.push_back("block_2");

        virtual_world.removeCollisionObjects(object_ids);
    }
}
   
        
