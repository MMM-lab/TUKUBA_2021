#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// HajimeCart parameter
const double wheel_radius = 0.095f;   // wr
const double base_width   = 0.340f;   // ws

// HajimeCart parameter
const double wheel_radius           = 0.095f;   
const double base_width             = 0.340f; 
const float wheel_to_wheel_distance = 0.310f;

// ロータリーエンコーダの1回転あたりのパルス数
int rotary_encoder_resolution = 100;

// ロータリーエンコーダの回転数を保持. 1回転で400増加
int encoder_value_left  = 0;
int encoder_value_right = 0;

// 現時刻でのロータリーエンコーダの角度を保持
float rotation_angle_left  = 0.0f;
float rotation_angle_right = 0.0f;

// 前時刻でのロータリーエンコーダの角度を保持
float pre_rotation_angle_left = 0.0f;
float pre_rotation_angle_right = 0.0f;

// 現時刻でのロータリーエンコーダの角速度を保持
float rotation_angular_velocity_left  = 0.0f;
float rotation_angular_velocity_right = 0.0f;

// 両車輪の移動速度
float left_vel  = 0.0f;
float right_vel = 0.0f;

// ロボットの並進速度
float linear_vel = 0.0f;
// ロボットの回転速度
float angular_vel = 0.0f;

// 左右のモータへの電圧
float delta_speed_left  = 0.0f;
float delta_speed_right = 0.0f;
float i_left  = 0.0f;
float i_right = 0.0f;
float ki = 0.00001;
float kp = 1;
float motor_left_value  = 0.0f;
float motor_right_value = 0.0f;

// モーター出力電圧に加えるオフセット値(摩擦トルクを相殺するために用いる)
float motor_offset = 0.17; //volt

// PWM出力のパルス幅(0~100で指定する)
int pwm_width_left  = 0;
int pwm_width_right = 0;

// first flag
int flag_firsttime = 1;

// ros time
ros::Publisher odom_pub;
ros::Publisher pwm_pub;
ros::Time current_time;
ros::Time last_time;

std_msgs::Float32MultiArray cmdmotorspeed;

std_msgs::Int32MultiArray pwm_data;

void cmd_vel_callback(const geometry_msgs::Twist& msg) 
{
  cmdmotorspeed.data[0]  = (msg.linear.x - msg.angular.z * base_width / 2.0) / wheel_radius;    // left motor [rad/s]
  cmdmotorspeed.data[1]  = (msg.linear.x + msg.angular.z * base_width / 2.0) / wheel_radius;    // right motor [rad/s]
}

void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    encoder_value_left  = msg->data[0];
    encoder_value_right = msg->data[1];

    if( flag_firsttime )
    {
        flag_firsttime = 0;

     	  last_time = ros::Time::now();

        return;
    }

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

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

    /**
     ***********************************************************************
     * Moter Control
     ***********************************************************************
    */
  
    // P制御
    delta_speed_left  = cmdmotorspeed.data[0] - rotation_angular_velocity_left;
    //delta_speed_right = -cmdmotorspeed.data[1] - rotation_angular_velocity_right;
    delta_speed_right = cmdmotorspeed.data[1] - rotation_angular_velocity_right;
    // I制御
    i_left += delta_speed_left * dt;
    i_right += delta_speed_right * dt;
    // PI制御
    motor_left_value  = kp * delta_speed_left + ki * i_left;
    motor_right_value = kp * delta_speed_right + ki * i_right;

    // motor_left_value
    if(motor_left_value > 0)
    {
        motor_left_value += motor_offset;   
    }
    if(motor_left_value < 0)
    {
        motor_left_value -= motor_offset;    
    }

    // motor_right_value
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

    pwm_data.data[0] = pwm_width_left;
    pwm_data.data[1] = pwm_width_right; 
    pwm_pub.publish(&pwm_data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmdvel_cmdmotorspeed");
  ros::NodeHandle nh;

  cmdmotorspeed.data.resize(2);
  cmdmotorspeed.data[0] = 0.0;
  cmdmotorspeed.data[1] = 0.0;

  ros::Publisher cmdmotorspeed_pub = nh.advertise<std_msgs::Float32MultiArray>("cmdmotorspeed", 10);
  ros::Publisher pwm_pub = nh.advertise<std_msgs::Int32MultiArray>("pwm", 10);
  ros::Subscriber encoder_sub = nh.subscribe("encoder", 10, encoderCallback);
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmd_vel_callback);

  ros::Rate loop_rate(100);	//100Hz(10ms)

  while (ros::ok()) {
    cmdmotorspeed_pub.publish(cmdmotorspeed);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}