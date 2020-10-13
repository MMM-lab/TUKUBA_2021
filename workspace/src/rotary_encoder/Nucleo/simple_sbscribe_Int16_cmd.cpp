/* NUCLEO */
#include "mbed.h"

/* ROS */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>

/* ROS communication*/
ros::NodeHandle nh;

float motor_left_ref;
float motor_right_ref;

// cmd_vel
void motor_speed_Cb(const std_msgs::Int16MultiArray& msg)
{
  motor_left_ref  = (float) (msg.data[0] / 1000);
  motor_right_ref = (float) (msg.data[1] / 1000);
}

ros::Subscriber<std_msgs::Int16MultiArray> cmdmotorspeed_sub("cmdmotorspeed", &motor_speed_Cb);

void ros_init()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(cmdmotorspeed_sub);
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