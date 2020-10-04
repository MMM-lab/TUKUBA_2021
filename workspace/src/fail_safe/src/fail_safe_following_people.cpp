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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fail_safe_following_people_node");

    ros::NodeHandle nh;
    odom_sub    = nh.subscribe("odom", 10, odom_callback);
    scan_sub    = nh.subscribe("scan", 10, scan_callback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate rate(20.0);

    while (ros::ok())
    {
        ros::spinOnce();

        if (latest_scan.ranges.size() > 0)
        {

            float hx = 0.0f;
            float hy = 0.0f;
            int hnum = 0.0f;

            // theta-range 座標系から x-y 座標系に変換
            for(unsigned int i = 0; i < latest_scan.ranges.size(); i ++)
            {
                if (!(latest_scan.ranges[i] < latest_scan.range_min ||
                    latest_scan.ranges[i] > latest_scan.range_max ||
                    std::isnan(latest_scan.ranges[i])))
                {

                    float theta = latest_scan.angle_min + i * latest_scan.angle_increment;
                    // x-y 座標系に変換
                    float x = latest_scan.ranges[i] * cosf(theta);
                    float y = latest_scan.ranges[i] * sinf(theta);

                    if (fabs(y) < 0.5 && x > 0.05 && x < 1.05)
                    {
                        // ロボット正面方向、1m四方の領域にある点の重心を計算
                        hx += x;
                        hy += y;
                        hnum ++;
                    }
                }
            }
            
            if (hnum > 0) {
                hy /= hnum;
                hx /= hnum;
                float dir  = atan2(hy, hx);
                float dist = hypotf(hy, hx);

                ROS_INFO("Following the object in front (dist %0.3f, dir %0.3f)", dist, dir);

                // 角加速度0.6rad/ssで、向きを最短時間制御
                cmd_vel.angular.z = sqrtf(2.0 * 0.6 * fabs(dir));

                if (dir < 0) {
                    cmd_vel.angular.z *= -1;
                }
                if (cmd_vel.angular.z > 0.4) {
                    cmd_vel.angular.z = 0.4;
                } else if (cmd_vel.angular.z < -0.4) {
                    cmd_vel.angular.z = -0.4;
                }

                // 加速度0.3m/ssで、距離を最短時間制御
                cmd_vel.linear.x = sqrtf(2.0 * 0.3 * fabs(dist - 0.5));

                if (dist - 0.5 < 0) {
                    cmd_vel.linear.x *= -1;
                }
                if (cmd_vel.linear.x > 0.2) {
                    cmd_vel.linear.x = 0.2;
                } else if (cmd_vel.linear.x < -0.2) {
                    cmd_vel.linear.x = -0.2;
                }
            }

            cmd_vel_pub.publish(cmd_vel);
        }

        rate.sleep();
    }
  
    return 0;
}