#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <vector>
#include <fstream>
#include <string>

ros::Subscriber sub; 
nav_msgs::Odometry odom; 
std::vector<geometry_msgs::Pose> waypoints;
geometry_msgs::PoseStamped finish_pose_;
geometry_msgs::Pose pre_pose;
std::string filename_ = "waypoints.yaml";
int start;

void odomCallback(const nav_msgs::Odometry &odom_msg) 
{ 
    ROS_INFO("odom : x %lf  : y %lf\n", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y); 

    if(pow(odom_msg.pose.pose.position.x - pre_pose.position.x, 2.0) + 
        pow(odom_msg.pose.pose.position.y - pre_pose.position.y, 2.0) + 
        pow(odom_msg.pose.pose.position.z - pre_pose.position.z, 2.0) >= 0.5) {

        if(start == 0) {
            start=1;
        }

        waypoints.push_back(odom_msg.pose.pose);

        ROS_INFO_STREAM("save current position");

        pre_pose.position.x    = odom_msg.pose.pose.position.x;
        pre_pose.position.y    = odom_msg.pose.pose.position.y;
        pre_pose.position.z    = odom_msg.pose.pose.position.z;
        pre_pose.orientation.x = odom_msg.pose.pose.orientation.x;
        pre_pose.orientation.y = odom_msg.pose.pose.orientation.y;
        pre_pose.orientation.z = odom_msg.pose.pose.orientation.z;
        pre_pose.orientation.w = odom_msg.pose.pose.orientation.w;
    }

    // update finish position
    finish_pose_.header.seq = 0;
    //finish_pose_.header.stamp = 0.0;//pre_timestamp;
    finish_pose_.header.frame_id    = "odom";
    finish_pose_.pose.position.x    = odom_msg.pose.pose.position.x;
    finish_pose_.pose.position.y    = odom_msg.pose.pose.position.y;
    finish_pose_.pose.position.z    = odom_msg.pose.pose.position.z;
    finish_pose_.pose.orientation.x = odom_msg.pose.pose.orientation.x;
    finish_pose_.pose.orientation.y = odom_msg.pose.pose.orientation.y;
    finish_pose_.pose.orientation.z = odom_msg.pose.pose.orientation.z;
    finish_pose_.pose.orientation.w = odom_msg.pose.pose.orientation.w;

    return; 
} 

void save()
{
    std::ofstream ofs(filename_.c_str(), std::ios::out);

    ofs << "waypoints:" << std::endl;

    for(int i = 0; i < waypoints.size(); i++){
        ofs << "    " << "- position:" << std::endl;
        ofs << " position_x: " << waypoints[i].position.x << std::endl;
        ofs << " position_y: " << waypoints[i].position.y << std::endl;
        ofs << " position_z: " << waypoints[i].position.z << std::endl;
        ofs << " orientation_x: "<< waypoints[i].orientation.x << std::endl;
        ofs << " orientation_y: "<< waypoints[i].orientation.y << std::endl;
        ofs << " orientation_z: "<< waypoints[i].orientation.z << std::endl;
        ofs << " orientation_w: "<< waypoints[i].orientation.w << std::endl;
    }

    ofs << "    " << "- position:" << std::endl;
    ofs << " position_x: " << finish_pose_.pose.position.x << std::endl;
    ofs << " position_y: " << finish_pose_.pose.position.y << std::endl;
    ofs << " position_z: " << finish_pose_.pose.position.z << std::endl;
    ofs << " orientation_x: "<< finish_pose_.pose.orientation.x << std::endl;
    ofs << " orientation_y: "<< finish_pose_.pose.orientation.y << std::endl;
    ofs << " orientation_z: "<< finish_pose_.pose.orientation.z << std::endl;
    ofs << " orientation_w: "<< finish_pose_.pose.orientation.w << std::endl;

    ofs.close();

    std::cout << "write success"<<std::endl;
}

void odom_loop() { 
    ros::Rate loop_rate(2); //2Hz 

    while(ros::ok()) { 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 

    if(start != 0){  //save waypoints
        ROS_INFO_STREAM("start write");
        save();
    }

    return; 
} 

int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "odom_listener"); 

    ros::NodeHandle n; 
    sub = n.subscribe("odom", 1, odomCallback);

    start = 0;
    pre_pose.position.x    = 0.0;
    pre_pose.position.y    = 0.0;
    pre_pose.position.z    = 0.0;
    pre_pose.orientation.x = 0.0;
    pre_pose.orientation.y = 0.0;
    pre_pose.orientation.z = 0.0;
    pre_pose.orientation.w = 0.0;

    odom_loop(); 

    return 0; 
}