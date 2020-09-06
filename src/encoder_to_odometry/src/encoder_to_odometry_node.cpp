#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// HajimeCart parameter
const double wheel_radius       = 0.085;    // wr
const double base_width         = 0.236;    // ws
const double _pi                = 3.141592654;
const double xp_enc_ticks_meter = 6250.0 / (2.0 * _pi * wheel_radius);

// encoder
double pos_x             = 0.0;
double pos_y             = 0.0;
double pos_th            = 0.0;
double encoder_left      = 0.0;
double encoder_right     = 0.0;
double encoder_left_old  = 0.0;
double encoder_right_old = 0.0;

// first flag
int flag_firsttime = 1;

// callback function
void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);

/* ROS */
ros::Publisher odom_pub;
// 現時刻と前時刻を保存して離散時間dtを算出
ros::Time current_time;
ros::Time last_time;

/* main */
int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;

    ros::Subscriber encoder_sub = n.subscribe("encoder", 10, encoderCallback);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::spin(); 

    return 0;
}

// callback function
void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    static tf::TransformBroadcaster odom_broadcaster;

    encoder_left  = msg->data[0];
    encoder_right = msg->data[1];

    if (flag_firsttime) {
        flag_firsttime = 0;

        encoder_left_old  = encoder_left;
        encoder_right_old = encoder_right;

	    last_time = ros::Time::now();

        return;
    }

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    //double dt=((double)current_time.sec + 1e-9*(double)current_time.nsec) - ((double)last_time.sec + 1e-9*(double)last_time.nsec);
    // 離散時間dtを算出
    double dt      = (current_time - last_time).toSec();
    
    double d_left  = (encoder_left -  encoder_left_old ) / xp_enc_ticks_meter;
    double d_right = (encoder_right -  encoder_right_old ) / xp_enc_ticks_meter;
    
    encoder_left_old  = encoder_left;
    encoder_right_old = encoder_right;

    double d  = (d_left + d_right) / 2.0;
    double th = (d_right - d_left) / base_width;

    // calc dt becomes big for the first time
    double dx;
    double dr;
    if (dt < 1000.0) {
        dx = d / dt;
        dr = th / dt;
    } else {
        dt = 0.0;
        dx = 0;
        dr = 0;
    }

    if (fabs(d) > 1.0e-5) {
        double _x = cos(th) * d;
        double _y = -sin(th) * d;
        pos_x += (_x * cos(pos_th) - _y * sin(pos_th));
        pos_y += (_x * sin(pos_th) + _y * cos(pos_th));
    }

    if (fabs(th) > 1.0e-5) {
        pos_th += th;
    }

    // const double wr= 0.085;
    // const double ws = 0.236;
    //    r =  ws/2.0 * (encoder_left + encoder_right)/(encoder_right - encoder_left );
    //  omega_dt = (encoder_right - encoder_left) * step / ws
    //  iccx = x - r * sin(theta)
    //  iccy = y + r * cos(theta)
    //  x_new = (x - iccx) * cos(omega_dt) - (y - iccy) * sin(omega_dt) + iccx;
    //  y_new = (x - iccx) * sin(omega_dt) + (y - iccy) * cos(omega_dt) + iccy;
    //  theta_new = theta + omega_dt;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos_th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp            = current_time;
    odom_trans.header.frame_id         = "/odom";
    odom_trans.child_frame_id          = "/base_footprint";
    odom_trans.transform.translation.x = pos_x;
    odom_trans.transform.translation.y = pos_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation      = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp          = current_time;
    odom.header.frame_id       = "/odom";
    odom.pose.pose.position.x  = pos_x;
    odom.pose.pose.position.y  = pos_y;
    odom.pose.pose.position.z  = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id        = "/base_footprint";
    odom.twist.twist.linear.x  = dx;
    odom.twist.twist.linear.y  = 0;
    odom.twist.twist.angular.z = dr;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}