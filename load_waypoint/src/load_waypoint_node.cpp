#include <ros/ros.h>
#include <ros/package.h>
#include "load_waypoint/load_waypoint.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "load_waypoint"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated load_waypoint node");
    ros::Duration r(0.5);

    LOAD_WAYPOINT waypoints(n);

    waypoints.initialization();
    while(ros::ok()){
        waypoints.waypoints_pub();

        r.sleep();    
    }
    
    ros::spinOnce();
    return 0;
}