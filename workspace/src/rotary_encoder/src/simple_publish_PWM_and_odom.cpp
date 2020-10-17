#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

/***********************************************************************
 * Global variables
 **********************************************************************/
// HajimeCart parameter
const double wheel_radius           = 0.095f;   
const double base_width             = 0.340f; 
const float wheel_to_wheel_distance = 0.310f;

// ロータリーエンコーダの1回転あたりのパルス数
int rotary_encoder_resolution = 100;

// ロータリーエンコーダの回転数を保持. 1回転で400増加
int encoder_value_left;
int encoder_value_right;

// 現時刻でのロータリーエンコーダの角度を保持
float rotation_angle_left;
float rotation_angle_right;

// 前時刻でのロータリーエンコーダの角度を保持
float pre_rotation_angle_left = 0.0f;
float pre_rotation_angle_right = 0.0f;

// 現時刻でのロータリーエンコーダの角速度を保持
float rotation_angular_velocity_left;
float rotation_angular_velocity_right;

// 両車輪の移動速度
float left_vel  = 0.0f;
float right_vel = 0.0f;

// ロボットの並進速度
float linear_vel = 0.0f;
// ロボットの回転速度
float angular_vel = 0.0f;

// ロボットの位置と姿勢
float x     = 0.0f;
float y     = 0.0f;
float theta = 0.0f;

ros::Publisher odom_pub;
ros::Publisher cmdmotorspeed_pub;
ros::Time last_time;
ros::Time current_time;

// odom
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;
//tf::TransformBroadcaster odom_broadcaster;

// 制御周期
float dt; //sec

/***********************************************************************
 * Moter Control variables
 **********************************************************************/
// 目標トルク
float target_left_torque;
float target_right_torque;
// 目標電流
float target_left_electric_current;
float target_right_electric_current;
// 慣性モーメント(本体とモータとタイヤ全体の慣性モーメントを求める)
float I = 10.0;
// ギア比(減速比)
float ita = 64.8;
// モーターの回転速度
float omega_left;
float omega_right;
// モータのトルク定数
float kt = 0.0018;
// モーターの逆起電力定数
float kb = 0.0024;
// 回路全体の抵抗
float R = 2.4;

// PWM出力のパルス幅(0~100で指定する)
int pwm_width_left;
int pwm_width_right;
std_msgs::Int8MultiArray pwm_width_data;

// 左右のモータへの電圧
float delta_speed_left;
float delta_speed_right;
float i_left;
float i_right;
float ki = 0.00001;
float kp = 1;
float motor_left_value;
float motor_right_value;

// モーター出力電圧に加えるオフセット値(摩擦トルクを相殺するために用いる)
float motor_offset = 0.17; //volt

/***********************************************************************
 * Callback function
 **********************************************************************/
float target_rotation_angular_velocity_left;
float target_rotation_angular_velocity_right;

// 参考資料 : https://hajimerobot.co.jp/ros/robotcart/
/*void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  rotation_angular_velocity_left  = (msg.linear.x - msg.angular.z * base_width / 2.0) / wheel_radius;    // left motor [rad/s]
  rotation_angular_velocity_right = (msg.linear.x + msg.angular.z * base_width / 2.0) / wheel_radius;    // right motor [rad/s]
}*/

// 参考資料 : https://at-wat.github.io/ROS-quick-start-up/files/Vehicle_and_Motion_Control.pdf
void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  target_rotation_angular_velocity_left  = (2 * msg.linear.x - msg.angular.z * base_width) / (2 * wheel_radius);    // left motor [rad/s]
  target_rotation_angular_velocity_right = (2 * msg.linear.x + msg.angular.z * base_width) / (2 * wheel_radius);    // right motor [rad/s]
}

void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    encoder_value_left  = msg->data[0];
    encoder_value_right = msg->data[1];
}

/**
 *****************************************************************************************************
 * MAIN METHOD
 *****************************************************************************************************
*/
int main(int argc, char** argv) {
  ros::init(argc, argv, "cmdvel_cmdmotorspeed");
  ros::NodeHandle nh;
  
  pwm_width_data.data.resize(2);
  pwm_width_data.data[0] = 0;
  pwm_width_data.data[1] = 0;

  // publisher
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  //cmdmotorspeed_pub = nh.advertise<std_msgs::Int32MultiArray>("cmdmotorspeed", 10);
  cmdmotorspeed_pub = nh.advertise<std_msgs::Int8MultiArray>("cmdmotorspeed", 10);

  // sbscriber
  ros::Subscriber encoder_sub = nh.subscribe("encoder", 10, encoderCallback);
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmd_vel_callback);
  
  // time 
  last_time    = ros::Time::now();
  current_time = ros::Time::now();

  ros::Rate loop_rate(10);	//10Hz(10ms)

  while (ros::ok()) {
    current_time = ros::Time::now();

    dt = (current_time - last_time).toSec();

    /**
    ***********************************************************************
    * Odometry
    ***********************************************************************
    */
    //-------------------------------------------
    // タイヤの回転角
    //-------------------------------------------
    rotation_angle_left  = encoder_value_left * (2 * 3.14f)/(4 * rotary_encoder_resolution);
    rotation_angle_right = encoder_value_right * (2 * 3.14f)/(4 * rotary_encoder_resolution);

    //-------------------------------------------
    // タイヤの回転速度
    //-------------------------------------------
    rotation_angular_velocity_left  = (rotation_angle_left - pre_rotation_angle_left) / dt;
    rotation_angular_velocity_right = (rotation_angle_right - pre_rotation_angle_right) / dt;

    //-------------------------------------------
    // ロボットの移動速度
    //-------------------------------------------
    left_vel    = rotation_angular_velocity_left * wheel_radius;
    right_vel   = rotation_angular_velocity_right * wheel_radius;
    linear_vel  = (right_vel + left_vel) / 2.0f;
    angular_vel = (right_vel - left_vel) / wheel_to_wheel_distance;

    //-------------------------------------------
    // ロボットの位置
    //-------------------------------------------
    x     = x + linear_vel * dt * cos(theta + angular_vel / 2.0f);
    y     = y + linear_vel * dt * sin(theta + angular_vel / 2.0f);
    theta = theta + angular_vel * dt;

    static tf::TransformBroadcaster odom_broadcaster;
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    odom_trans.header.stamp    = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation      = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x  = x;
    odom.pose.pose.position.y  = y;
    odom.pose.pose.position.z  = 0.0;
    odom.pose.pose.orientation = odom_quat ;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x  = linear_vel;
    odom.twist.twist.angular.z = angular_vel;

    /**
    ***********************************************************************
    * Moter Control
    ***********************************************************************
    */
    
    /*// P制御
    delta_speed_left  = rotation_angular_velocity_left - rotation_angular_velocity_left;
    delta_speed_right = rotation_angular_velocity_right - rotation_angular_velocity_right;
    // I制御
    i_left += delta_speed_left * dt;
    i_right += delta_speed_right * dt;
    // PI制御
    motor_left_value  = kp * delta_speed_left + ki * i_left;
    motor_right_value = kp * delta_speed_right + ki * i_right;*/

    //-------------------------------------------
    // PI制御でモータのトルク制御
    //-------------------------------------------
    // P制御
    delta_speed_left  = target_rotation_angular_velocity_left - rotation_angular_velocity_left;
    delta_speed_right = target_rotation_angular_velocity_right - rotation_angular_velocity_right;
    // I制御
    i_left += delta_speed_left * dt;
    i_right += delta_speed_right * dt;
    // PI制御
    target_left_torque  = kp * I * delta_speed_left + ki * I * i_left;
    target_right_torque = kp * I * delta_speed_right + ki * I * i_right;

    //-------------------------------------------
    // 電圧のPWM変換
    //-------------------------------------------
    // 電流の計算
    target_left_electric_current  = target_left_torque / kt;
    target_right_electric_current = target_right_torque / kt;
    // モータの回転速度の計算
    omega_left  = ita * rotation_angular_velocity_left;
    omega_right = ita * rotation_angular_velocity_right;
    // 電圧の計算
    motor_left_value  = R * target_left_electric_current + kb * omega_left;
    motor_right_value = R * target_right_electric_current + kb * omega_right;
    // オフセット値を足す
    if(motor_left_value > 0)
    {
        motor_left_value += motor_offset;   
    }
    if(motor_left_value < 0)
    {
        motor_left_value -= motor_offset;    
    }
    if(motor_right_value > 0)
    {
        motor_right_value += motor_offset;   
    }
    if(motor_right_value < 0)
    {
        motor_right_value -= motor_offset;    
    }
    //calculate PWM pulse width
    pwm_width_left  = int(motor_left_value * 100.0f / 3.3f );
    pwm_width_right = int(motor_right_value * 100.0f / 3.3f );
    pwm_width_data.data[0] = pwm_width_left;
    pwm_width_data.data[1] = pwm_width_right;

    //publish the message
    odom_pub.publish(odom);
    cmdmotorspeed_pub.publish(pwm_width_data);

    last_time = current_time;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}