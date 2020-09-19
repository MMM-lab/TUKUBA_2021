//=========================================================
//MPU board:  NUCLEO-F401RE
//Motor driver: TA7291P x 2
//=========================================================
/*  Nucleo-F401RE   */
#include "mbed.h"
#include "math.h"

/* serial communication */
Serial pc(USBTX, USBRX);

/* ROS */
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

ros::NodeHandle nh;
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;
ros::Publisher odom_pub("odom", &odom);
tf::TransformBroadcaster odom_broadcaster;

//=========================================================
// タイヤの半径
const float wheel_radius = 0.0925f;
// 車輪感距離
const float wheel_to_wheel_distance = 0.25f;
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

//=========================================================
// ロータリーエンコーダの出力を受けるためにBusInクラスを使用して
// I/Oポートを入力として設定
BusIn encoder_bus(PC_11, PD_2); //Encoder (LSB to MSB)
// モータードライバ
DigitalOut IN1(PC_8);   //TA7291P IN1
DigitalOut IN2(PC_6);   //TA7291P IN2
// モーター制御のためのPWM出力
PwmOut motor(PC_9);     //TA7291P Vref

//=========================================================
// タイマー割込み処理
// ロータリーエンコーダの値を監視するポーリング処理
Ticker timer1; //for rotary encoder

//=========================================================
// ロータリーエンコーダに関する変数
// ロータリーエンコーダの出力を確認する時間間隔
int rotary_encoder_update_rate = 25; //usec
// ロータリーエンコーダの1回転あたりのパルス数
int rotary_encoder_resolution = 100;
// ロータリーエンコーダの回転数を保持. 1回転で400増加
int encoder_value = 0;
// ロータリーエンコーダの位相を求めるためのテーブル
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};
// 現時刻でのロータリーエンコーダの角度を保持
float rotation_angle;
// 前時刻でのロータリーエンコーダの角度を保持
float pre_rotation_angle = 0.0;
// 現時刻でのロータリーエンコーダの角速度を保持
float rotation_angular_velocity;

//=========================================================
// モーターの制御周期
float feedback_rate = 0.01; //sec
// 計算によって求めたモーターの出力電圧(今回は決め値)
float motor_value = 0;
// PWM出力のパルス幅(0~100で指定する)
int pwm_width = 0;
// モーターの回転方向(1なら正転, 2なら反転)
int motor_direction = 1;
// モーター出力電圧に加えるオフセット値(摩擦トルクを相殺するために用いる)
float motor_offset = 0.17; //volt

//=========================================================
//Rotary encoder polling function
//It takes 4usec. (NUCLEO-F401RE 84MHz)
//=========================================================
// ロータリーエンコーダの角度を測定
void rotary_encoder_check()
{  
    static int code; 
    //check the movement
    code = ( (code<<2) +  int(encoder_bus) ) & 0xf ;
    //update the encoder value
    int value = -1 * table[code];
    encoder_value += value;
    
    return;
}

void ros_init()
{
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(odom_pub);
    odom_broadcaster.init(nh);
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
    // Motor driver intialization
    //-------------------------------------------
    IN1 = 0; //motor stop
    IN2 = 0; //motor stop
    motor.period_us(100);   //10 kHz pulse
    motor.pulsewidth_us(0); //0 to 100

    //-------------------------------------------  
    // Timer
    //-------------------------------------------  
    //timer1: rotary encoder polling, 40 kHz
    timer1.attach_us(&rotary_encoder_check, rotary_encoder_update_rate);

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
        // タイヤの回転角
        rotation_angle            = encoder_value * (2*3.14f)/(4*rotary_encoder_resolution);
        // タイヤの回転速度
        rotation_angular_velocity = (rotation_angle - pre_rotation_angle) / feedback_rate;
        /* デッドレコニング(オドメトリ) */
        // ロボットの移動速度
        left_vel    = rotation_angular_velocity * wheel_radius;
        right_vel   = rotation_angular_velocity * wheel_radius;
        linear_vel  = (right_vel + left_vel) / 2.0f;
        angular_vel = (right_vel - left_vel) / wheel_to_wheel_distance;
        // ロボットの位置
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
        //pc.printf("%f\r%f\r%d\r", rotation_angle, rotation_angular_velocity, encoder_value);
        // ロボットの位置
        //pc.printf("%f\r%f\r%f\r\n", x, y, theta);

        /**
         ***********************************************************************
         * Moter Control
         ***********************************************************************
        */
        // 本来この値は何かしらの制御則によって算出されるべき
        motor_value = 0.5f;
        
        //offset
        if(motor_value > 0)
        {
            motor_value += motor_offset;   
        }
        if(motor_value < 0)
        {
            motor_value -= motor_offset;    
        }
        
        //calculate PWM pulse width
        pwm_width = int( motor_value*100.0f/3.3f );
        
        //drive the motor in forward
        if(pwm_width>=0)
        {
            //over voltage
            if(pwm_width>100)
            {
                pwm_width = 100;    
            }         
            //to protect TA7291P
            if(motor_direction == 2)
            {
                /* TA7291Pのデータシートには, モータの回転方向を
                反転させる場合は100usだけIN1 = 0, IN2 = 0とする
                デッドタイムを入れて回路が燃えるのを防ぐ必要がある*/ 
                IN1 = 0;
                IN2 = 0;
                wait(0.0001); //wait 100 usec    
            }        
            //forward
            IN1 = 1;
            IN2 = 0;
            
            motor.pulsewidth_us(pwm_width);
            motor_direction = 1;
        }      
        //drive the motor in reverse
        else
        {
            //calculate the absolute value
            pwm_width = -1 * pwm_width;

            //over voltage
            if(pwm_width>100)
            {
                pwm_width = 100;    
            }
            //to protect TA7291P
            if(motor_direction == 1)
            {
                /* TA7291Pのデータシートには, モータの回転方向を
                反転させる場合は100usだけIN1 = 0, IN2 = 0とする
                デッドタイムを入れて回路が燃えるのを防ぐ必要がある*/ 
                IN1 = 0;
                IN2 = 0;
                wait(0.0001); //wait 100 usec    
            }
            //reverse
            IN1 = 0;
            IN2 = 1;

            motor.pulsewidth_us(pwm_width);
            motor_direction = 2;          
        }
        // prepare for the next calculation of theta2_dot
        pre_rotation_angle = rotation_angle;
        // wait 
        nh.spinOnce();  
        wait(feedback_rate);
        wait_ms(1000);    
    }    
}