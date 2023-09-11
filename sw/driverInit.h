#ifndef __DRIVERINIT_H
#define __DRIVERINIT_H

#include "stdint.h"//包含uint32_t
#include "unistd.h"//包含usleep()
#include "fcntl.h"
#include "stdio.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/mman.h"//包含内存映射地址函数mmap

#include "include/hwlib.h"
#include "hps_0.h"
#include "include/socal/hps.h"
#include "include/socal/socal.h"
#include "include/socal/alt_gpio.h"

#include "motor.h"

//有关内存映射的一些必要定义
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )   //内存页大小
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )//网关?与该值做与运算,能够保证数值不超过该值

//PWM寄存器的相对偏移地址
#define PWM_ENABLE    	2   //PWM的使能设置寄存器地址
#define PWM_FREQUENCY   0   //PWM的频率设置寄存器地址
#define PWM_DUTYCYCLE   1   //PWM的占空比设置寄存器地址


int driverInit();
int driverClose();
int turnOnTB6612FNG(int state);

#endif

/*
extern int fd_driver;
extern void *virtual_base;	//L3在内存中的映射地址

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
*/