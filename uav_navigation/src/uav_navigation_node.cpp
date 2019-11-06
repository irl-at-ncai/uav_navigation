#include <ros/ros.h>
#include "uav_navigation/global_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_navigation_node");
    uav_navigation::GlobalPlanner global_planner;  
    auto rate = ros::Rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}