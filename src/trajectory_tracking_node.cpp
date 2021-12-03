#include <ros/ros.h>
#include "crazyflie_show/crazyflie_show_one.h"

crazyflie_trajectory_tracking::CrazyflieShow* crazyflie_show;

void landCallback(const std_msgs::EmptyConstPtr & empty_msg){
    ROS_INFO_STREAM("Enter the landcallback");
    crazyflie_show->is_land_ = true;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_tracking");
    ros::NodeHandle nh, pnh("~");

    ros::Subscriber land_sub_ = nh.subscribe("/node_end_and_land", 1000, &landCallback);
    crazyflie_show = new crazyflie_trajectory_tracking::CrazyflieShow(nh, pnh);
    crazyflie_show->run();


    ros::MultiThreadedSpinner s(2);
    ros::spin(s);
}