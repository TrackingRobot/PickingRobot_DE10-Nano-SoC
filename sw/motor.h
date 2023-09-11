#ifndef __MOTOR_H
#define __MOTOR_H

#include "led.h"
#include "driverInit.h"
#include "inputdetect.h"
#include "node.h"

//PWM的占空比和脉冲
#define DUTY_PULSE  	3599
#define TOTAL_PULSE 	7199

//各电机方向设置
#define LMOTOR_GOAHEAD	0
#define LMOTOR_GOBACK	1

#define RMOTOR_GOAHEAD	0
#define RMOTOR_GOBACK	1

#define BODY_RAISE		0
#define	BODY_FALLING	1

#define	PICKER_UP		0
#define	PICKER_DOWN		1

#define FAN1_PUFF       1
#define FAN1_SNIFF 		0
#define	FAN2_PUFF		0
#define FAN2_SNIFF      1
//电机开机与关闭
#define TURN_ON			1
#define	TURN_OFF		0

enum CONTROL_CMD
{

		Stop=0,           //Car stop
    	TurnRight=1,
		TurnLeft,       //TurnLeft	
		GoAhead,        //Go forward 
		GoBack,         //Go Back 

		//捡球装置
		TurnDownPickerBody, //TurnDownPicker	
		HandOnPickerBody,   //HandOnPicker			
		TurnUpPickerBody,   //TurnUpPicker
		//捡球飞轮
		TurnUpFlyWheel,
        TurnOffFlyWheel,
		TurnDownFlyWheel,
		//风扇
		TurnOnFan,      //Turn On Wind		
		TurnOffFan      //Turn Off Wind			
		
};


//用来设置方向的IO
extern unsigned long *h2p_lw_led_addr;//led灯的地址
extern unsigned long *h2p_lw_LMotorDir_addr;	//L_MOTOR_dir的地址;
extern unsigned long *h2p_lw_RMotorDir_addr;	//R_MOTOR_dir的地址;
extern unsigned long *h2p_lw_PickerDir_addr;	//Picker_dir的地址;
extern unsigned long *h2p_lw_BodyDir_addr;		//BOdy_dir的地址;
extern unsigned long *h2p_lw_Fan1Dir_addr;		//Fan1_dir的地址;
extern unsigned long *h2p_lw_Fan2Dir_addr;		//Fan2_dir的地址;

//用来检测限位开关的传感器
extern unsigned long *h2p_lw_FalingIn_addr;	//falling_S_IN的地址;
extern unsigned long *h2p_lw_RaiseIn_addr;		//falling_S_IN的地址;

//用来管理程序的运行或暂停的IO
extern unsigned long *h2p_lw_StartOrPause_addr;

//用来管理PWM的IO的地址
extern unsigned long *h2p_lw_LPwmBase_addr;		//L_MOTOR的地址;
extern unsigned long *h2p_lw_RPwmBase_addr;		//R_MOTOR的地址;
extern unsigned long *h2p_lw_PickerPwmBase_addr;	//Picker_MOTOR的地址;	
extern unsigned long *h2p_lw_BodyPwmBase_addr;		//Body_MOTOR的地址;	
extern unsigned long *h2p_lw_Fan1PwmBase_addr;		//Fan1_MOTOR的地址;
extern unsigned long *h2p_lw_Fan2PwmBase_addr;		//Fan2_MOTOR的地址;

//各PWM使能设置的绝对地址							 							 
extern unsigned long *h2p_lw_LMotorPwmEnable_addr;
extern unsigned long *h2p_lw_RMotorPwmEnable_addr;
extern unsigned long *h2p_lw_PickerPwmEnable_addr;
extern unsigned long *h2p_lw_BodyPwmEnable_addr;
extern unsigned long *h2p_lw_Fan1PwmEnable_addr;
extern unsigned long *h2p_lw_Fan2PwmEnable_addr;

//PWM的占空比DutyCycle
extern unsigned long *h2p_lw_LPwmDutyCycle_addr;		//L_MOTOR的占空比;
extern unsigned long *h2p_lw_RPwmDutyCycle_addr;		//R_MOTOR的占空比;	
extern unsigned long *h2p_lw_PickerPwmDutyCycle_addr;	//Picker_MOTOR的占空比;
extern unsigned long *h2p_lw_BodyPwmDutyCycle_addr;	//Body_MOTOR的占空比;
extern unsigned long *h2p_lw_Fan1PwmDutyCycle_addr;	//Fan1_MOTOR的占空比;
extern unsigned long *h2p_lw_Fan2PwmDutyCycle_addr;	//Fan2_MOTOR的占空比;

//PWM的频率Frequency			
extern unsigned long *h2p_lw_LPwmFrequency_addr;		//L_MOTOR的频率;
extern unsigned long *h2p_lw_RPwmFrequency_addr;		//R_MOTOR的频率;	
extern unsigned long *h2p_lw_PickerPwmFrequency_addr;	//Picker_MOTOR的频率;
extern unsigned long *h2p_lw_BodyPwmFrequency_addr;	//Body_MOTOR的频率;
extern unsigned long *h2p_lw_Fan1PwmFrequency_addr;	//Fan1_MOTOR的频率;
extern unsigned long *h2p_lw_Fan2PwmFrequency_addr;	//Fan2_MOTOR的频率;

void LMotor(int direction,int enable,int dutycycle,int frequecy);
void RMotor(int direction,int enable,int dutycycle,int frequecy);
void PickerMotor(int direction,int enable,int dutycycle,int frequecy);
void BodyMotor(int direction,int enable,int dutycycle,int frequecy);
void Fan1Motor(int direction,int enable,int dutycycle,int frequecy);
void Fan2Motor(int direction,int enable,int dutycycle,int frequecy);


int SendControlCMDToCar(int command_num,int speedLeft,int speedRight);

void TestDriver();
int TestDriverDirection();
#endif