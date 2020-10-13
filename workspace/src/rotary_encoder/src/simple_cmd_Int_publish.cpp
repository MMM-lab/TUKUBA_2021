#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>

// HajimeCart parameter
const double wheel_radius = 0.095f;   // wr
const double base_width   = 0.340f;   // ws

std_msgs::Int16MultiArray cmdmotorspeed;

void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  cmdmotorspeed.data[0]  = (int16_t) ((msg.linear.x - msg.angular.z * base_width / 2.0) / wheel_radius) * 1000;    
  cmdmotorspeed.data[1]  = (int16_t) ((msg.linear.x + msg.angular.z * base_width / 2.0) / wheel_radius) * 1000;   
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmdvel_cmdmotorspeed");
  ros::NodeHandle nh;

  cmdmotorspeed.data.resize(2);
  cmdmotorspeed.data[0] = 0;
  cmdmotorspeed.data[1] = 0;

  ros::Publisher cmdmotorspeed_pub = nh.advertise<std_msgs::Int16MultiArray>("cmdmotorspeed", 10);

  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmd_vel_callback);

  ros::Rate loop_rate(10);	

  while (ros::ok()) {
    cmdmotorspeed_pub.publish(cmdmotorspeed);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}