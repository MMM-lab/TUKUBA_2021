#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_vel_publish");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::Rate loop_rate(10);	

  while (ros::ok()) {
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x  = 1.0;
    cmd_msg.angular.z = 0.0;

    cmd_pub.publish(cmd_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}