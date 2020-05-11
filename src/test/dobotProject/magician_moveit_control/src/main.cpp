#include <planning/planning.h>
using namespace std;
int main(int argc,char** argv)
{
    ros::init(argc,argv,"action_client");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    my_planning::MyPlanningClass plan(node);
   
    plan.goToPoseGoal();
    //plan.addObjects();
   
    //plan.cartesianPath();
     
    //plan.move_group.move();
    //spinner.stop();
    while(1);
}