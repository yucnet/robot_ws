#include "handeye/handeye.hpp"
#include <pthread.h>

using namespace std;
 
void* thread( void* )
{
    ros::Rate r(60);
    ros::Time current_time;
    while ( ros::ok() )
    {
        pthread_mutex_lock(&mutex);
        current_time = ros::Time::now();
        ros::spinOnce();
        r.sleep();
        pthread_mutex_unlock(&mutex);
    }
}
   

int main(int argc, char** argv)
{
   
    ros::init(argc, argv, "hand_eye"); 
    ros::NodeHandle n;
    Handeye x(n);

    sleep(3);

    //创建回调函数线程,回调函数开始工作
    pthread_t callback_t;
    pthread_create(&callback_t, NULL, thread, NULL);

    cout<<"机械臂进入工作"<<endl;
    // // {
    // //     /* code */
    // pthread_mutex_lock(&mutex);
    //x.isworking = 1;
    // pthread_mutex_unlock(&mutex);

    // x.process( hsvblue, "blue");
    // x.cameraAxisCalculation("blue");

    // cout<<"dianshu: "<<x.targetpoints["blue"].size()<<endl;

    // for ( int i = 0; x.targetpoints["blue"].size(); i++ ) {

    //     x.goToPoint( x.targetpoints["blue"][i].x, x.targetpoints["blue"][i].y, x.targetpoints["blue"][i].z );
        
    // }

    while(1);

}

