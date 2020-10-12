/*
  *******************************************************************************
 * @file PI control pwm
 * @brief cmd_vel to pwm value
 * @author Ramune6110
 * @date 2020 10/06
 * SYSTEM            | NUCLEO-F401RE
 * Motor driver      | TA7291P x 2
 * Rotary encoder    | EC202A100A x 2
 * publish           | encoder
 * sbscribe          | cmdmotorspeed
 * sbscribe          | rotation_angular_velocity
  *******************************************************************************
*/
/***********************************************************************
 * Header files 
 **********************************************************************/
/* NUCLEO */
#include "mbed.h"

/* ROS */
#include <ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

/***********************************************************************
 * Global variables
 **********************************************************************/

/* ROS communication*/
ros::NodeHandle nh;

float motor_left_ref  = 0.0f;
float motor_right_ref = 0.0f;
/*float rotation_angular_velocity_left  = 0.0f;
float rotation_angular_velocity_right = 0.0f;*/

/*void motor_speed_Cb(const std_msgs::Float32MultiArray& msg)
{
  motor_left_ref  = msg.data[0];
  motor_right_ref = msg.data[1];
}

void rotation_vel_Cb(const std_msgs::Float32MultiArray& msg)
{
  rotation_angular_velocity_left  = msg.data[0];
  rotation_angular_velocity_right = msg.data[1];
}*/

// publish
/*std_msgs::Int32MultiArray encoder_data;
ros::Publisher encoder_pub("encoder", &encoder_data);*/

// subscribe
//ros::Subscriber<std_msgs::Float32MultiArray> cmdmotorspeed_sub("cmdmotorspeed", &motor_speed_Cb);
//ros::Subscriber<std_msgs::Float32MultiArray> rotation_angular_velocity_sub("rotation_angular_velocity", &rotation_vel_Cb);

// PWM
// PWM出力のパルス幅(0~100で指定する)
int pwm_width_left;
int pwm_width_right;
//void motor_speed_Cb(const std_msgs::Int32MultiArray& msg)
//{
//  pwm_width_left  = msg.data[0];
//  pwm_width_right = msg.data[1];
//}
//
//ros::Subscriber<std_msgs::Int32MultiArray> cmdmotorspeed_sub("cmdmotorspeed", &motor_speed_Cb);

/*void motor_speed_Cb(const std_msgs::Int8MultiArray& msg)
{
  pwm_width_left  = msg.data[0];
  pwm_width_right = msg.data[1];
}

ros::Subscriber<std_msgs::Int8MultiArray> cmdmotorspeed_sub("cmdmotorspeed", &motor_speed_Cb);*/

// odom
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;
ros::Publisher odom_pub("odom", &odom);

// HajimeCart parameter
const double wheel_radius           = 0.095f;   
const double base_width             = 0.340f; 
const float wheel_to_wheel_distance = 0.310f;
// cmd_vel
void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  motor_left_ref  = (msg.linear.x - msg.angular.z * base_width / 2.0) / wheel_radius;    // left motor [rad/s]
  motor_right_ref = (msg.linear.x + msg.angular.z * base_width / 2.0) / wheel_radius;    // right motor [rad/s]
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_callback);

/***********************************************************************
 * Global variables
 **********************************************************************/

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

/***********************************************************************
 * Encoder variables
 **********************************************************************/
//=========================================================
/* 左側のモーター設定 */
BusIn encoder_bus_left(PC_11, PD_2); //Encoder (LSB to MSB)
// モータードライバ
DigitalOut IN1_left(PC_8);   //TA7291P IN1_left
DigitalOut IN2_left(PC_6);   //TA7291P IN2_left
// モーター制御のためのPWM出力
PwmOut motor_left(PC_9);     //TA7291P Vref

/* 右側のモーター設定 */
BusIn encoder_bus_right(PC_10, PC_12); //Encoder (LSB to MSB)
// モータードライバ
DigitalOut IN1_right(PB_8);   //TA7291P IN1_left
DigitalOut IN2_right(PC_5);   //TA7291P IN2_left
// モーター制御のためのPWM出力
PwmOut motor_right(PB_9);     //TA7291P Vref

//=========================================================
// タイマー割込み処理
// ロータリーエンコーダの値を監視するポーリング処理
Ticker timer_encoder; //for rotary encoder

//=========================================================
// ロータリーエンコーダの出力を確認する時間間隔
int rotary_encoder_update_rate = 10000; //usec

// ロータリーエンコーダの回転数を保持. 1回転で400増加
//int encoder_value_left  = 0;
//int encoder_value_right = 0;

// ロータリーエンコーダの位相を求めるためのテーブル
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};

//=========================================================
// モーターの制御周期
float feedback_rate = 0.01; //sec
float dt = 1.0f;

// モーターの回転方向(1なら正転, 2なら反転)
int motor_direction_left  = 1;
int motor_direction_right = 1;

// モーター出力電圧に加えるオフセット値(摩擦トルクを相殺するために用いる)
float motor_offset = 0.17; //volt

// 左右のモータへの電圧
float delta_speed_left  = 0.0f;
float delta_speed_right = 0.0f;
float i_left  = 0.0f;
float i_right = 0.0f;
float ki = 0.00001f;
float kp = 1.0f;
float motor_left_value  = 0.0f;
float motor_right_value = 0.0f;

/**
 ***********************************************************************
 * Rotary encoder polling function
 * It takes 4usec. (NUCLEO-F401RE 84MHz)
 ***********************************************************************
*/
// ロータリーエンコーダの角度を測定
void rotary_encoder_check()
{  
    static int code_left; 
    static int code_right; 
    //check the movement
    code_left  = ( (code_left<<2) + int(encoder_bus_left) ) & 0xf ;
    code_right = ( (code_right<<2) + int(encoder_bus_right) ) & 0xf ;
    //update the encoder value
    int value_left  = -1 * table[code_left];
    int value_right = -1 * table[code_right];

    encoder_value_left += value_left;
    encoder_value_right += value_right;

    // encoder publish
    /*encoder_data.data[0] = encoder_value_left;
    encoder_data.data[1] = encoder_value_right; 
    encoder_pub.publish(&encoder_data);*/
}

void ros_init()
{
    /*encoder_data.data_length = 2;
    encoder_data.data = (int32_t *)malloc(sizeof(int32_t)*2);
    encoder_data.data[0] = 0.0f;
    encoder_data.data[1] = 0.0f;*/

    nh.getHardware()->setBaud(921600);
    nh.initNode();
    //nh.advertise(encoder_pub);
    nh.advertise(odom_pub);
    //nh.subscribe(cmdmotorspeed_sub);
    nh.subscribe(cmd_vel_sub);
    //nh.subscribe(rotation_angular_velocity_sub);
}

/**
 *****************************************************************************************************
 * MAIN METHOD
 *****************************************************************************************************
*/
int main() {   
    /***********************************************************************
     * Initialization
     **********************************************************************/
    //-------------------------------------------
    // System initialization
    //-------------------------------------------
    ros_init();
     
    //-------------------------------------------
    // Motor driver intialization
    //-------------------------------------------
    IN1_left  = 0; //motor stop
    IN2_left  = 0; //motor stop
    IN1_right = 0; //motor stop
    IN2_right = 0; //motor stop
    motor_left.period_us(10);    //10 kHz pulse
    motor_right.period_us(10);   //10 kHz pulse
    motor_left.pulsewidth_us(0);  //0 to 100
    motor_right.pulsewidth_us(0); //0 to 100

    //-------------------------------------------  
    // Timer
    //-------------------------------------------  
    //timer_left: rotary encoder polling, 40 kHz
    timer_encoder.attach_us(&rotary_encoder_check, rotary_encoder_update_rate);
    
    /**
     ***********************************************************************
     * main loop
     ***********************************************************************
    */
    while(1)
    {
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
        //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //first, we'll publish the transform over tf
        odom_trans.header.stamp    = nh.now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id  = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation      = tf::createQuaternionFromYaw(theta);

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        odom.header.stamp = nh.now();
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x  = x;
        odom.pose.pose.position.y  = y;
        odom.pose.pose.position.z  = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x  = linear_vel;
        odom.twist.twist.angular.z = angular_vel;

        /**********************************************************************
        * PI Moter Control
        **********************************************************************/
        // P制御
        delta_speed_left  = motor_left_ref - rotation_angular_velocity_left;
        //delta_speed_right = -cmdmotorspeed.data[1] - rotation_angular_velocity_right;
        delta_speed_right = motor_right_ref - rotation_angular_velocity_right;
        // I制御
        i_left += delta_speed_left * feedback_rate;
        i_right += delta_speed_right * feedback_rate;
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

        /***********************************************************************
        * left moter PWM
        **********************************************************************/
        //drive the motor in forward
        if(pwm_width_left>=0)
        {
            //over voltage
            if(pwm_width_left>100)
            {
                pwm_width_left = 100;    
            }         
            //to protect TA7291P
            if(motor_direction_left == 2)
            {
                IN1_left = 0;
                IN2_left = 0;
                wait(0.0001); //wait 100 usec    
            }        
            //forward
            IN1_left = 1;
            IN2_left = 0;
            
            motor_left.pulsewidth_us(pwm_width_left);
            motor_direction_left = 1;
        }      
        //drive the motor in reverse
        else
        {
            //calculate the absolute value
            pwm_width_left = -1 * pwm_width_left;

            //over voltage
            if(pwm_width_left>100)
            {
                pwm_width_left = 100;    
            }
            //to protect TA7291P
            if(motor_direction_left == 1)
            {
                IN1_left = 0;
                IN2_left = 0;
                wait(0.0001); //wait 100 usec    
            }
            //reverse
            IN1_left = 0;
            IN2_left = 1;

            motor_left.pulsewidth_us(pwm_width_left);
            motor_direction_left = 2;          
        }

        /***********************************************************************
        * right moter PWM
        **********************************************************************/
        //drive the motor in forward
        if(pwm_width_right>=0)
        {
            //over voltage
            if(pwm_width_right>100)
            {
                pwm_width_right = 100;    
            }         
            //to protect TA7291P
            if(motor_direction_right == 2)
            {
                IN1_right = 0;
                IN2_right = 0;
                wait(0.0001); //wait 100 usec    
            }        
            //forward
            IN1_right = 1;
            IN2_right = 0;
            
            motor_right.pulsewidth_us(pwm_width_right);
            motor_direction_right = 1;
        }      
        //drive the motor in reverse
        else
        {
            //calculate the absolute value
            pwm_width_right = -1 * pwm_width_right;

            //over voltage
            if(pwm_width_right>100)
            {
                pwm_width_right = 100;    
            }
            //to protect TA7291P
            if(motor_direction_right == 1)
            {
                IN1_right = 0;
                IN2_right = 0;
                wait(0.0001); //wait 100 usec    
            }
            //reverse
            IN1_right = 0;
            IN2_right = 1;

            motor_right.pulsewidth_us(pwm_width_right);
            motor_direction_right = 2;          
        }
        
        //publish the message
        odom_pub.publish(&odom);

        // wait 
        //__disable_irq();
        // encoder_data.data[0] = encoder_value_left;
        // encoder_data.data[1] = encoder_value_right; 
        // encoder_pub.publish(&encoder_data);
        nh.spinOnce();  
        //__enable_irq();
        wait_ms(1000);    
    }    
}