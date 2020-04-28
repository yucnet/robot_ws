#include <planning/planning.h>

namespace my_planning
{
    void MyPlanningClass::goToPoseGoal()
    {
        move_group.setPoseTarget(this->target_pose1);
        bool success  = (move_group.plan(this->my_plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
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
         





    }

}

