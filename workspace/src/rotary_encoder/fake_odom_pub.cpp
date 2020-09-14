#include "mbed.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

ros::NodeHandle n;
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;
ros::Publisher odom_pub("odom", &odom);
tf::TransformBroadcaster odom_broadcaster;

int main(int argc, char** argv){
  n.initNode();
  n.advertise(odom_pub);
  odom_broadcaster.init(n);

  while(1){
    //first, we'll publish the transform over tf
    odom_trans.header.stamp = n.now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = 1.0;
    odom_trans.transform.translation.y = 2.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation      = tf::createQuaternionFromYaw(1.57);

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = n.now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x  = 1.0;
    odom.pose.pose.position.y  = 2.0;
    odom.pose.pose.position.z  = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(1.57);

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x  = 3.0;
    odom.twist.twist.linear.y  = 4.0;
    odom.twist.twist.angular.z = 5.0;

    //publish the message
    odom_pub.publish(&odom);

    n.spinOnce();   
    wait_ms(1000);
  }
}