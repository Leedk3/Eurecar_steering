#include <ros/ros.h>
#include <ros/package.h>
#include "usrg_path_tracking/quintic_polynomials.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "usrg_path_tracker"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated usrg_path_tracker node");
    ros::Rate r(50);

    POLY_FIT path_follower(n);
    while(ros::ok()){

        path_follower.main_seqeunce();
        // path_follower.reference_path_fitting();
        ros::spinOnce();
        r.sleep();
    }

    
    // ros::spinOnce();
    return 0;
}