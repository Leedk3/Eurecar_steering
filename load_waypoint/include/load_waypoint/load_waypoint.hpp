#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <math.h>
#include <iostream>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LOAD_WAYPOINT
{
    public:
        LOAD_WAYPOINT(ros::NodeHandle& n);        
        ~LOAD_WAYPOINT();
        void initialization();
        void waypoints_pub();

    private:        
        // initialize variables

        ros::NodeHandle nh;
        ros::Publisher wpt_poses_pub;
        geometry_msgs::PoseArray wpt_poses;

        // initialize variables
        std::vector <std::pair<double, double>> waypointVect;
        std::vector<std::pair < double, double> > ::iterator iter; //init. iterator
        std::string path_abs;
        // std::string path_local = "/waypoint_files/utm_points.txt";
        std::string path_local = nh.param<std::string>("/load_waypoint_node/import_file", "/waypoint_files/utm_points.txt");
        double numWaypoints ;
        int count = 0;

        geometry_msgs::PoseArray interpolation_2d(std::vector <std::pair<double, double>> &waypointVect);
        int countWaypointsInFile(std::string path_local);
        std::vector <std::pair<double, double>> getWaypoints(std::string path_local);

        //ros parameter
};

LOAD_WAYPOINT::LOAD_WAYPOINT(ros::NodeHandle& n) 
{
    wpt_poses_pub = nh.advertise<geometry_msgs::PoseArray>("/PoseArray/wpt", 10);
    ROS_DEBUG("vehicle marker publisher created");
};

LOAD_WAYPOINT::~LOAD_WAYPOINT() 
{    
    ROS_INFO("LOAD_WAYPOINT destructor.");
}

void LOAD_WAYPOINT::initialization(){
    // Count number of waypoints
    // ROS_INFO("%s", path_local);
    numWaypoints = countWaypointsInFile(path_local);
    // Reading waypoints from text file and output results
    waypointVect = getWaypoints(path_local);
    wpt_poses = interpolation_2d(waypointVect);
}


void LOAD_WAYPOINT::waypoints_pub()
{
    wpt_poses_pub.publish(wpt_poses);
}

geometry_msgs::PoseArray LOAD_WAYPOINT::interpolation_2d(std::vector <std::pair<double, double>> &waypointVect)
{
    geometry_msgs::PoseArray interpolated_poses;
    
    int size = waypointVect.size();
    double step_dist = 0.0; //distance of each referential(given in the file) waypoint
    double inter_size = 0.5; //size of each LOAD_WAYPOINTd points
    int step_idx = 0; //the number of index between waypoints
    tf2::Quaternion quat_ekf;
    geometry_msgs::Quaternion quat_ekf_msg;

    // std::vector<double> x_array, y_array;
    // x_array.clear();
    // y_array.clear();
    double x_j = 0;
    double y_j = 0;
    double head_j = 0;
    double x_j_prev = 0;
    double y_j_prev = 0;
    interpolated_poses.poses.clear();
    interpolated_poses.header.stamp = ros::Time::now();
    interpolated_poses.header.frame_id = "/odom";
    for (int i = 0; i < size-1 ; i++){
        step_dist = sqrt((waypointVect[i+1].first-waypointVect[i].first)*(waypointVect[i+1].first-waypointVect[i].first)
                        +(waypointVect[i+1].second-waypointVect[i].second)*(waypointVect[i+1].second-waypointVect[i].second));
        step_idx = floor(step_dist / inter_size);
        for (int j = 0; j < step_idx ; j++){
            x_j_prev = x_j;
            y_j_prev = y_j;
            x_j = waypointVect[i].first +  j * (waypointVect[i+1].first - waypointVect[i].first) / step_idx;
            y_j = waypointVect[i].second + j * (waypointVect[i+1].second - waypointVect[i].second) / step_idx;
            head_j = atan2(y_j-y_j_prev,x_j-x_j_prev);
            if (std::isnan(head_j)){
                head_j = 0;
            }
            quat_ekf.setRPY(0,0,head_j);
            quat_ekf.normalize();
            quat_ekf_msg = tf2::toMsg(quat_ekf);

            // x_array.push_back(x_j);
            // y_array.push_back(y_j);
            geometry_msgs::Pose pose;
            pose.position.x = x_j;
            pose.position.y = y_j;
            pose.orientation = quat_ekf_msg;
            interpolated_poses.poses.push_back(pose);
        }
    }

    return interpolated_poses;
}

int LOAD_WAYPOINT::countWaypointsInFile(std::string path_local)
{
    path_abs = ros::package::getPath("load_waypoint") + path_local;
    ROS_INFO("%s is opened" , path_abs.c_str());
    std::ifstream fileCount(path_abs.c_str());
    if(fileCount.is_open())
    {
        double lati = 0;
        while(!fileCount.eof())
        {
            fileCount >> lati;
            ++count;
        }
        count = count - 1;
        numWaypoints = count / 2;
        ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
        fileCount.close();
    }
    else
    {
        std::cout << "Unable to open waypoint file" << std::endl;
        ROS_ERROR("Unable to open waypoint file");
    }
    return numWaypoints;
}

std::vector <std::pair<double, double>> LOAD_WAYPOINT::getWaypoints(std::string path_local)
{
    double lati = 0, longi = 0;

    path_abs = ros::package::getPath("load_waypoint") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    for(int i = 0; i < numWaypoints; i++)
    {
        fileRead >> lati;
        fileRead >> longi;
        waypointVect.push_back(std::make_pair(lati, longi));
    }
    fileRead.close();

    //Outputting vector
    ROS_INFO("The following GPS Waypoints have been set:");
    for(std::vector < std::pair < double, double >> ::iterator iterDisp = waypointVect.begin(); iterDisp != waypointVect.end();
    iterDisp++)
    {
        ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
    }
    return waypointVect;
}
