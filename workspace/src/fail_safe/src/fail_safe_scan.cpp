#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

ros::Publisher cmd_vel_pub;
ros::Subscriber odom_sub;
ros::Subscriber scan_sub;

nav_msgs::Odometry odom;
geometry_msgs::Twist cmd_vel;
sensor_msgs::LaserScan latest_scan;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    latest_scan = *msg;
}

// Monitoring +5° in front
int8_t observationFront_positive_5_degree()
{
    int count = 0;

    for(int i = 0; i < 5; i++) {
        /**
        ***********************************************************************
        * First condition  |  In case of an error value
        * Second condition |  Out of measurement range
        * Third condition  |  For infinity
        ***********************************************************************
        */
        if (latest_scan.ranges[i] < latest_scan.range_min || 
            latest_scan.ranges[i] > latest_scan.range_max || 
            std::isnan(latest_scan.ranges[i])) 
        {
            // 測定範囲外 = 正面に十分な距離があるわけではない。(場合によっては、とても近い距離かもしれない)
            ROS_INFO("i = %d, front-range: measurement error", i);
        } 
        else 
        {
            ROS_INFO("i = %d, front-range: %0.3f", i, latest_scan.ranges[i]);
            if (latest_scan.ranges[i] > 0.2) {
                // 20cm以上距離がある
                ;
            } else {
                // 20cm以下になった
                count += 1;
            }
        }
    }

    return count;
}

// Monitoring -5° in front
int8_t observationFront_negative_5_degree()
{
    int count = 0;

    for(int i = 355; i < 360; i++) {
        /**
        ***********************************************************************
        * First condition  |  In case of an error value
        * Second condition |  Out of measurement range
        * Third condition  |  For infinity
        ***********************************************************************
        */
        if (latest_scan.ranges[i] < latest_scan.range_min || 
            latest_scan.ranges[i] > latest_scan.range_max || 
            std::isnan(latest_scan.ranges[i])) 
        {
            // 測定範囲外 = 正面に十分な距離があるわけではない。(場合によっては、とても近い距離かもしれない)
            ROS_INFO("i = %d, front-range: measurement error", i);
        } 
        else 
        {
            ROS_INFO("i = %d, front-range: %0.3f", i, latest_scan.ranges[i]);
            if (latest_scan.ranges[i] > 0.2) {
                // 20cm以上距離がある
                ;
            } else {
                // 20cm以下になった
                count += 1;
            }
        }
    }

    return count;
}

// Monitoring ±5° in front
int8_t obstacle_detection()
{
    int8_t positive_count = observationFront_positive_5_degree();
    int8_t negative_count = observationFront_negative_5_degree();

    return positive_count + negative_count;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fail_safe_scan_node");

    ros::NodeHandle nh;
    odom_sub    = nh.subscribe("odom", 10, odom_callback);
    scan_sub    = nh.subscribe("scan", 10, scan_callback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate rate(10.0);

    while(ros::ok()) {
        ros::spinOnce();

        if (latest_scan.ranges.size() > 0) {
            // move_baseで計算された値を使用
            /*cmd_vel.linear.x  = 0.2;
            cmd_vel.angular.z = 0.0;*/
            if(obstacle_detection() >= 3) {
                ROS_INFO("STOP!!");
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = 0.0;

                cmd_vel_pub.publish(cmd_vel);
            }
        }

        rate.sleep();
    }

    return 0;
}

