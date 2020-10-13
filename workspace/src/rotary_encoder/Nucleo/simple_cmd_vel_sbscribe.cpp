/* NUCLEO */
#include "mbed.h"

/* ROS */
#include <ros.h>
#include <geometry_msgs/Twist.h>

/* ROS communication*/
ros::NodeHandle nh;

float linear_x;
float angler_z;

// cmd_vel
void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  linear_x = msg.linear.x;
  angler_z = msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmd_vel_callback);

void ros_init()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(cmd_sub);
}

int main()
{
    ros_init();
    
    while (1) 
    {
        nh.spinOnce();  
        wait_ms(10);   
    }
}