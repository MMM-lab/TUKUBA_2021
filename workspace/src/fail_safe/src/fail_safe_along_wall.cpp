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
float left_prev;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    latest_scan = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fail_safe_along_wall_node");

    ros::NodeHandle nh;
    odom_sub    = nh.subscribe("odom", 10, odom_callback);
    scan_sub    = nh.subscribe("scan", 10, scan_callback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    left_prev = FLT_MAX;

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        ros::spinOnce();

        if (latest_scan.ranges.size() > 0)
        {
            float front = FLT_MAX;
            float left  = FLT_MAX;

            // theta-range 座標系から x-y 座標系に変換
            for(unsigned int i = 0; i < latest_scan.ranges.size(); i ++) {
                if (!(latest_scan.ranges[i] < latest_scan.range_min ||
                    latest_scan.ranges[i] > latest_scan.range_max ||
                    std::isnan(latest_scan.ranges[i])))
                {

                    float theta = latest_scan.angle_min + i * latest_scan.angle_increment;
                    // x-y 座標系に変換
                    float x = latest_scan.ranges[i] * cosf(theta);
                    float y = latest_scan.ranges[i] * sinf(theta);

                    if (fabs(y) < 0.25 && x > 0.05) {
                        // ロボット正面方向のデータについて最小距離を計算
                        if (front > x) front = x;
                    } else if (fabs(x) < 0.25 && y > 0.25) {
                        // ロボット左方向のデータについて最小距離を計算
                        if (left > y) left = y;
                    }
                }
            }

            if (front > 0.5) {
                ROS_INFO("Following left wall (distance %0.3f)", left);
                cmd_vel.linear.x = 0.1;

                if (left > 1.0) {
                    left = 1.0;
                }

                // 角速度指令値を0に近づけるようにフィードバック
                cmd_vel.angular.z += -cmd_vel.angular.z * 0.01;
                // 左方向の距離を0.5mに近づけるようにフィードバック
                cmd_vel.angular.z += (left - 0.5) * 0.02;
                // 距離の変化量(壁の向きを表す)を0に近づけるようにフィードバック
                
                if (left_prev < 1.0) {
                    cmd_vel.angular.z += (left - left_prev) * 4.0;
                }
                
                if (cmd_vel.angular.z > 0.3) {
                    cmd_vel.angular.z = 0.3;
                } else if (cmd_vel.angular.z < -0.3) {
                    cmd_vel.angular.z = -0.3;
                } 

            } else {
                ROS_INFO("Something in front");
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -0.2;
            }

            left_prev = left;

            cmd_vel_pub.publish(cmd_vel);
        }

        rate.sleep();
    }

    return 0;
}