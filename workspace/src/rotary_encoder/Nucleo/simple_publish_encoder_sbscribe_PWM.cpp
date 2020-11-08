/*
  *******************************************************************************
 * @file PI control pwm
 * @brief sub encoder pub PWM
 * @author Ramune6110
 * @date 2020 11/08
 * SYSTEM            | NUCLEO-F401RE
 * Motor driver      | TA7291P x 2
 * Rotary encoder    | EC202A100A x 2
 * publish           | encoder
 * sbscribe          | cmdmotorspeed(PWM value)
  *******************************************************************************
*/
/***********************************************************************
 * Header files 
 **********************************************************************/
/* NUCLEO */
#include "mbed.h"

/* ROS */
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int64MultiArray.h>

/***********************************************************************
 * Global variables
 **********************************************************************/
/* ROS communication*/
ros::NodeHandle nh;

// subscribe PWM 
// PWM出力のパルス幅(0~100で指定する)
int pwm_width_left;
int pwm_width_right;

void motor_speed_Cb(const std_msgs::Int8MultiArray& msg)
{
  pwm_width_left  = msg.data[0];
  pwm_width_right = msg.data[1];
}

ros::Subscriber<std_msgs::Int8MultiArray> cmdmotorspeed_sub("cmdmotorspeed", &motor_speed_Cb);

// publish
std_msgs::Int64MultiArray encoder_data;
ros::Publisher encoder_pub("encoder", &encoder_data);

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
// ロータリーエンコーダの回転数を保持. 1回転で400増加
int encoder_value_left  = 0;
int encoder_value_right = 0;

// ロータリーエンコーダの位相を求めるためのテーブル
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};

//=========================================================
// モーターの回転方向(1なら正転, 2なら反転)
int motor_direction_left  = 1;
int motor_direction_right = 1;

void ros_init()
{
    encoder_data.data_length = 2;
    encoder_data.data = (int64_t *)malloc(sizeof(int64_t)*2);
    encoder_data.data[0] = 0;
    encoder_data.data[1] = 0;

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(encoder_pub);
    nh.subscribe(cmdmotorspeed_sub);
}

/**
 *****************************************************************************************************
 * MAIN METHOD
 *****************************************************************************************************
*/
int main() {   
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
    motor_left.period_us(100);    //10 kHz pulse
    motor_right.period_us(100);   //10 kHz pulse
    motor_left.pulsewidth_us(0);  //0 to 100
    motor_right.pulsewidth_us(0); //0 to 100
    
    /**
     ***********************************************************************
     * main loop
     ***********************************************************************
    */
    while(1)
    {
        //-------------------------------------------
        // Encoder data
        //-------------------------------------------
        static int code_left; 
        static int code_right; 
        //check the movement
        code_left  = ( (code_left<<2) + int(encoder_bus_left) ) & 0xf ;
        code_right = ( (code_right<<2) + int(encoder_bus_right) ) & 0xf ;
        //update the encoder value
        int value_left  = -1 * table[code_left];
        //int value_right = -1 * table[code_right];
        int value_right = table[code_right];
        
        encoder_value_left += value_left;
        encoder_value_right += value_right;

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

        // encoder publish
        encoder_data.data[0] = encoder_value_left;
        encoder_data.data[1] = encoder_value_right; 
        encoder_pub.publish(&encoder_data);

        nh.spinOnce();  
        wait_ms(10);    
    }    
}