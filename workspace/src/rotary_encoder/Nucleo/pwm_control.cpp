/*
  *******************************************************************************
 * @file PI control pwm
 * @brief cmd_vel to pwm value
 * @author Ramune6110
 * @date 2020 10/06
 * SYSTEM            | NUCLEO-F401RE
 * Motor driver      | TA7291P x 2
 * Rotary encoder    | EC202A100A x 2
 * publish           | odom
 * sbscribe          | cmd_vel
  *******************************************************************************
*/
/***********************************************************************
 * Header files 
 **********************************************************************/
/* NUCLEO */
#include "mbed.h"
#include "math.h"

/* ROS */
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

/***********************************************************************
 * Global variables
 **********************************************************************/

/* serial communication */
Serial pc(USBTX, USBRX);

/* ROS communication*/
ros::NodeHandle nh;

// odom
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;
tf::TransformBroadcaster odom_broadcaster;
ros::Publisher odom_pub("odom", &odom);

// cmd_vel
// cmd_vel to rad/s
std_msgs::Float32MultiArray cmdmotorspeed;
void cmd_vel_callback(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist>  cmd_vel_sub("cmd_vel", &cmd_vel_callback);

/***********************************************************************
 * Robot variables
 **********************************************************************/
//=========================================================
// タイヤの半径
const float wheel_radius = 0.095f;

// 車輪間距離
const float wheel_to_wheel_distance = 0.310f;

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

//const double wheel_radius= 0.085;   // wr
const float base_width = 0.340;    // ws

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
Ticker timer_left; //for rotary encoder
Ticker timer_right; //for rotary encoder

//=========================================================
// ロータリーエンコーダの出力を確認する時間間隔
int rotary_encoder_update_rate = 25; //usec

// ロータリーエンコーダの1回転あたりのパルス数
int rotary_encoder_resolution = 100;

// ロータリーエンコーダの回転数を保持. 1回転で400増加
int encoder_value_left  = 0;
int encoder_value_right = 0;

// ロータリーエンコーダの位相を求めるためのテーブル
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};

// 現時刻でのロータリーエンコーダの角度を保持
float rotation_angle_left  = 0.0f;
float rotation_angle_right = 0.0f;

// 前時刻でのロータリーエンコーダの角度を保持
float pre_rotation_angle_left = 0.0;
float pre_rotation_angle_right = 0.0;

// 現時刻でのロータリーエンコーダの角速度を保持
float rotation_angular_velocity_left  = 0.0f;
float rotation_angular_velocity_right = 0.0f;

//=========================================================
// モーターの制御周期
float feedback_rate = 0.01; //sec

// 計算によって求めたモーターの出力電圧(今回は決め値)
float motor_value = 0;

// PWM出力のパルス幅(0~100で指定する)
int pwm_width_left  = 0;
int pwm_width_right = 0;

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
float ki = 0.00001;
float kp = 1;
float motor_left_value  = 0.0f;
float motor_right_value = 0.0f;

/**
 ***********************************************************************
 * Rotary encoder polling function
 * It takes 4usec. (NUCLEO-F401RE 84MHz)
 ***********************************************************************
*/
// ロータリーエンコーダの角度を測定
void rotary_encoder_check_left()
{  
    static int code; 
    //check the movement
    code = ( (code<<2) +  int(encoder_bus_left) ) & 0xf ;
    //update the encoder value
    int value = -1 * table[code];
    encoder_value_left += value;
    
    return;
}

void rotary_encoder_check_right()
{  
    static int code; 
    //check the movement
    code = ( (code<<2) +  int(encoder_bus_right) ) & 0xf ;
    //update the encoder value
    int value = -1 * table[code];
    encoder_value_right += value;
    
    return;
}

void ros_init()
{
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(odom_pub);
    nh.subscribe(cmd_vel_sub);
    odom_broadcaster.init(nh);
}

/**
 ***********************************************************************
 * cmd_vel
 ***********************************************************************
*/
void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  cmdmotorspeed.data[0]  = (msg.linear.x - msg.angular.z * base_width / 2.0) / wheel_radius;    // left motor [rad/s]
  cmdmotorspeed.data[1]  = (msg.linear.x + msg.angular.z * base_width / 2.0) / wheel_radius;    // right motor [rad/s]
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
    //pc.baud(115200);
    ros_init();
     
    //-------------------------------------------
    // cmd_vel
    //-------------------------------------------
    //cmdmotorspeed.data.resize(2);
    cmdmotorspeed.data[0] = 0.0;
    cmdmotorspeed.data[1] = 0.0;
    
    //-------------------------------------------
    // Motor driver intialization
    //-------------------------------------------
    IN1_left  = 0; //motor stop
    IN2_left  = 0; //motor stop
    IN1_right = 0; //motor stop
    IN2_right = 0; //motor stop
    motor_left.period_us(100);    //10 kHz pulse
    motor_right.period_us(100);   //10 kHz pulse
    motor_left.pulsewidth_us(0);  //0 to 100
    motor_right.pulsewidth_us(0); //0 to 100

    //-------------------------------------------  
    // Timer
    //-------------------------------------------  
    //timer_left: rotary encoder polling, 40 kHz
    timer_left.attach_us(&rotary_encoder_check_left, rotary_encoder_update_rate);
    timer_right.attach_us(&rotary_encoder_check_right, rotary_encoder_update_rate);

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
        rotation_angle_left  = encoder_value_left * (2*3.14f)/(4*rotary_encoder_resolution);
        rotation_angle_right = encoder_value_right * (2*3.14f)/(4*rotary_encoder_resolution);

        //-------------------------------------------
        // タイヤの回転速度
        //-------------------------------------------
        rotation_angular_velocity_left  = (rotation_angle_left - pre_rotation_angle_left) / feedback_rate;
        rotation_angular_velocity_right = (rotation_angle_right - pre_rotation_angle_right) / feedback_rate;

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
        x     = x + linear_vel * feedback_rate * cos(theta + angular_vel / 2.0f);
        y     = y + linear_vel * feedback_rate * sin(theta + angular_vel / 2.0f);
        theta = theta + angular_vel * feedback_rate;

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

        //publish the message
        odom_pub.publish(&odom);
        
        // タイヤの回転角と回転速度
        //pc.printf("%f\r%f\r%d\r", rotation_angle_left, rotation_angular_velocity_left, encoder_value);
        // ロボットの位置
        //pc.printf("%f\r%f\r%f\r\n", x, y, theta);

        /**
         ***********************************************************************
         * Moter Control
         ***********************************************************************
        */
        // 本来この値は何かしらの制御則によって算出されるべき
        motor_value = 0.5f;
        
        // P制御
        delta_speed_left  = cmdmotorspeed.data[0] - rotation_angular_velocity_left;
//        delta_speed_right = -cmdmotorspeed.data[1] - rotation_angular_velocity_right;
        delta_speed_right = cmdmotorspeed.data[1] - rotation_angular_velocity_right;
        // I制御
        i_left += delta_speed_left * feedback_rate;
        i_right += delta_speed_right * feedback_rate;
        // PI制御
        motor_left_value  = kp * delta_speed_left + ki * i_left;
        motor_right_value = kp * delta_speed_right + ki * i_right;
        
        //offset
        if(motor_value > 0)
        {
            motor_value += motor_offset;   
        }
        if(motor_value < 0)
        {
            motor_value -= motor_offset;    
        }

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
                /* TA7291Pのデータシートには, モータの回転方向を
                反転させる場合は100usだけIN1_left = 0, IN2_left = 0とする
                デッドタイムを入れて回路が燃えるのを防ぐ必要がある*/ 
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
                /* TA7291Pのデータシートには, モータの回転方向を
                反転させる場合は100usだけIN1_left = 0, IN2_left = 0とする
                デッドタイムを入れて回路が燃えるのを防ぐ必要がある*/ 
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
                /* TA7291Pのデータシートには, モータの回転方向を
                反転させる場合は100usだけIN1_left = 0, IN2_left = 0とする
                デッドタイムを入れて回路が燃えるのを防ぐ必要がある*/ 
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
                /* TA7291Pのデータシートには, モータの回転方向を
                反転させる場合は100usだけIN1_right = 0, IN2_right = 0とする
                デッドタイムを入れて回路が燃えるのを防ぐ必要がある*/ 
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

        // prepare for the next calculation of theta2_dot
        pre_rotation_angle_left  = rotation_angle_left;
        pre_rotation_angle_right = rotation_angle_right;

        // wait 
        nh.spinOnce();  
        wait_ms(1000);    
    }    
}