#include "driverInit.h"

int fd_driver=0;//定义内存设备号
void *virtual_base;	//L3在内存中的映射地址

//用于开启TB6612FNG芯片的管脚
unsigned long *h2p_lw_Motor_Tb6612_En;
unsigned long *h2p_lw_Fan_Tb6612_En;
unsigned long *h2p_lw_Pick_Body_Tb6612_En;

//用来设置方向的IO
unsigned long *h2p_lw_led_addr;			//led灯的地址
unsigned long *h2p_lw_LMotorDir_addr;	//L_MOTOR_dir的地址;
unsigned long *h2p_lw_RMotorDir_addr;	//R_MOTOR_dir的地址;
unsigned long *h2p_lw_PickerDir_addr;	//Picker_dir的地址;
unsigned long *h2p_lw_BodyDir_addr;		//BOdy_dir的地址;
unsigned long *h2p_lw_Fan1Dir_addr;		//Fan1_dir的地址;
unsigned long *h2p_lw_Fan2Dir_addr;		//Fan2_dir的地址;

//用来检测限位开关的传感器
unsigned long *h2p_lw_FalingIn_addr;	//falling_S_IN的地址;
unsigned long *h2p_lw_RaiseIn_addr;		//falling_S_IN的地址;

//用来管理程序的运行或暂停的IO
unsigned long *h2p_lw_StartOrPause_addr;
//用来管理程序终止或者运行的输入IO
unsigned long *h2p_lw_SwitchSW3_addr;

//用来管理PWM的IO的地址
unsigned long *h2p_lw_LPwmBase_addr;		//L_MOTOR的地址;
unsigned long *h2p_lw_RPwmBase_addr;		//R_MOTOR的地址;
unsigned long *h2p_lw_PickerPwmBase_addr;	//Picker_MOTOR的地址;	
unsigned long *h2p_lw_BodyPwmBase_addr;		//Body_MOTOR的地址;	
unsigned long *h2p_lw_Fan1PwmBase_addr;		//Fan1_MOTOR的地址;
unsigned long *h2p_lw_Fan2PwmBase_addr;		//Fan2_MOTOR的地址;

//各PWM使能设置的绝对地址							 							 
unsigned long *h2p_lw_LMotorPwmEnable_addr;
unsigned long *h2p_lw_RMotorPwmEnable_addr;
unsigned long *h2p_lw_PickerPwmEnable_addr;
unsigned long *h2p_lw_BodyPwmEnable_addr;
unsigned long *h2p_lw_Fan1PwmEnable_addr;
unsigned long *h2p_lw_Fan2PwmEnable_addr;

//PWM的占空比DutyCycle
unsigned long *h2p_lw_LPwmDutyCycle_addr;		//L_MOTOR的占空比;
unsigned long *h2p_lw_RPwmDutyCycle_addr;		//R_MOTOR的占空比;	
unsigned long *h2p_lw_PickerPwmDutyCycle_addr;	//Picker_MOTOR的占空比;
unsigned long *h2p_lw_BodyPwmDutyCycle_addr;	//Body_MOTOR的占空比;
unsigned long *h2p_lw_Fan1PwmDutyCycle_addr;	//Fan1_MOTOR的占空比;
unsigned long *h2p_lw_Fan2PwmDutyCycle_addr;	//Fan2_MOTOR的占空比;

//PWM的频率Frequency			
unsigned long *h2p_lw_LPwmFrequency_addr;		//L_MOTOR的频率;
unsigned long *h2p_lw_RPwmFrequency_addr;		//R_MOTOR的频率;	
unsigned long *h2p_lw_PickerPwmFrequency_addr;	//Picker_MOTOR的频率;
unsigned long *h2p_lw_BodyPwmFrequency_addr;	//Body_MOTOR的频率;
unsigned long *h2p_lw_Fan1PwmFrequency_addr;	//Fan1_MOTOR的频率;
unsigned long *h2p_lw_Fan2PwmFrequency_addr;	//Fan2_MOTOR的频率;


int driverInit()
{
	fd_driver=open("/dev/mem",(O_RDWR|O_SYNC));
 	
    if(-1== fd_driver)
    {
        printf("error !could not open \"/dev/mem\"******\n");
        return -1;       
    }
    //virtual_base为 HPS 的 L3 外设区域物理地址到虚拟地址
    virtual_base = mmap(NULL,HW_REGS_SPAN,(PROT_READ | PROT_WRITE),\
                            MAP_SHARED,fd_driver,HW_REGS_BASE );
    if(virtual_base == MAP_FAILED)
    {
        printf("error ! mmap() failed ... \n");
        close(fd_driver);
        return -1;
    }
	//ALT_LWFPGASLVS_OFST为轻量级 HPS-to-FPGA AXI 总线相对于 HPS 的 L3 外设区域地址的偏移地址
	//LED_PIO_BASE为pio_led 相对于轻量级 HPS-to-FPGA AXI 总线的偏移地址
	//SoC开发板LED灯地址绝对地址
    h2p_lw_led_addr = 		(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + LED_PIO_BASE)&\
                     		(unsigned long)(HW_REGS_MASK)));
	//用于开启TB6612FNG芯片的管脚
	h2p_lw_Motor_Tb6612_En	=(unsigned long *)((unsigned long)virtual_base + \
							(unsigned long)(ALT_LWFPGASLVS_OFST + TURNONMOTORPWM_BASE)&\
							(unsigned long)(HW_REGS_MASK));

	h2p_lw_Fan_Tb6612_En	=(unsigned long *)((unsigned long)virtual_base + \
							(unsigned long)(ALT_LWFPGASLVS_OFST + TURNONFANPWM_BASE)&\
							(unsigned long)(HW_REGS_MASK));
	h2p_lw_Pick_Body_Tb6612_En=(unsigned long *)((unsigned long)virtual_base + \
							(unsigned long)(ALT_LWFPGASLVS_OFST + TURNONBODY_PICKER_BASE)&\
							(unsigned long)(HW_REGS_MASK));							 
	//Raise和Falling两个传感器的输入
    h2p_lw_FalingIn_addr = 	(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + FALLING_S_IN_BASE)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_RaiseIn_addr = 	(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + RAISE_S_IN_BASE)&\
                     		(unsigned long)(HW_REGS_MASK)));
	//用来管理程序的运行或暂停的IO
	h2p_lw_StartOrPause_addr=(unsigned long *)( (unsigned long)virtual_base + \
							((unsigned long)(ALT_LWFPGASLVS_OFST + START_PAUSE_BASE)&\
							(unsigned long)(HW_REGS_MASK)) );
	//用来管理程序终止或者运行的输入IO
	h2p_lw_SwitchSW3_addr	=(unsigned long *)( (unsigned long)virtual_base + \
							((unsigned long)(ALT_LWFPGASLVS_OFST + DIPSW_PIO_BASE )&\
							(unsigned long)(HW_REGS_MASK)) );														 							 							 
	//各PWM设置方向的绝对地址							 
    h2p_lw_LMotorDir_addr	=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + L_MOTOR_DIR_BASE)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_RMotorDir_addr	=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + R_MOTOR_DIR_BASE)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_PickerDir_addr	=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + PICKER_DIR_BASE)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_BodyDir_addr		=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + BODY_DIR_BASE)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_Fan1Dir_addr		=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + FAN1_DIR_BASE)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_Fan2Dir_addr		=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + FAN2_DIR_BASE)&\
                     		(unsigned long)(HW_REGS_MASK)));

	//各PWM使能设置的绝对地址
    h2p_lw_LMotorPwmEnable_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + L_PWM_BASE + PWM_ENABLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_RMotorPwmEnable_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + R_PWM_BASE + PWM_ENABLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));								 							 
    h2p_lw_PickerPwmEnable_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + PICKER_PWM_BASE + PWM_ENABLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_BodyPwmEnable_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + BODY_PWM_BASE + PWM_ENABLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_Fan1PwmEnable_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + FAN1_PWM_BASE + PWM_ENABLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_Fan2PwmEnable_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + FAN2_PWM_BASE + PWM_ENABLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));

	//各PWM频率设置的绝对地址
    h2p_lw_LPwmFrequency_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + L_PWM_BASE + PWM_FREQUENCY * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_RPwmFrequency_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + R_PWM_BASE + PWM_FREQUENCY * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));								 							 
    h2p_lw_PickerPwmFrequency_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + PICKER_PWM_BASE + PWM_FREQUENCY * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_BodyPwmFrequency_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + BODY_PWM_BASE + PWM_FREQUENCY * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_Fan1PwmFrequency_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + FAN1_PWM_BASE + PWM_FREQUENCY * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_Fan2PwmFrequency_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + FAN2_PWM_BASE + PWM_FREQUENCY * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));

	//各PWM占空比设置绝对地址
    h2p_lw_LPwmDutyCycle_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + L_PWM_BASE + PWM_DUTYCYCLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_RPwmDutyCycle_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + R_PWM_BASE + PWM_DUTYCYCLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));								 							 
    h2p_lw_PickerPwmDutyCycle_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + PICKER_PWM_BASE + PWM_DUTYCYCLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_BodyPwmDutyCycle_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + BODY_PWM_BASE + PWM_DUTYCYCLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_Fan1PwmDutyCycle_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + FAN1_PWM_BASE + PWM_DUTYCYCLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    h2p_lw_Fan2PwmDutyCycle_addr=(unsigned long *)((unsigned long)virtual_base + \
                     		((unsigned long)(ALT_LWFPGASLVS_OFST + FAN2_PWM_BASE + PWM_DUTYCYCLE * 4)&\
                     		(unsigned long)(HW_REGS_MASK)));
    return 0;
}
int driverClose()
{
	SendControlCMDToCar(Stop,0,0);

	if(munmap(virtual_base,HW_REGS_SPAN) != 0)
	{
		printf( "ERROR: munmap() failed...\n" );
		close( fd_driver );
		return( 1 );
	}
}

int turnOnTB6612FNG(int state)
{
	printf("turnOnTB6612FNG>>>>>>>>>>>\n");
	*(uint32_t *)h2p_lw_Motor_Tb6612_En		= state;
	*(uint32_t *)h2p_lw_Fan_Tb6612_En		= state;
	*(uint32_t *)h2p_lw_Pick_Body_Tb6612_En	= state;

	return 0;
}