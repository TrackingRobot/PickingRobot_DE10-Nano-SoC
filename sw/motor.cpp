#include "motor.h"

void LMotor(int direction,int Enable,int dutycycle,int frequecy)
{
    *(uint32_t *)h2p_lw_LMotorDir_addr          = direction;
    *(uint32_t *)h2p_lw_LMotorPwmEnable_addr    = Enable;
    *(uint32_t *)h2p_lw_LPwmDutyCycle_addr      = dutycycle;
    *(uint32_t *)h2p_lw_LPwmFrequency_addr      = frequecy;
}

void RMotor(int direction,int Enable,int dutycycle,int frequecy)
{
    *(uint32_t *)h2p_lw_RMotorDir_addr          = direction;
    *(uint32_t *)h2p_lw_RMotorPwmEnable_addr  = Enable;
    *(uint32_t *)h2p_lw_RPwmDutyCycle_addr      = dutycycle;
    *(uint32_t *)h2p_lw_RPwmFrequency_addr      = frequecy;
}

void PickerMotor(int direction,int Enable,int dutycycle,int frequecy)
{
    *(uint32_t *)h2p_lw_PickerDir_addr          = direction;
    *(uint32_t *)h2p_lw_PickerPwmEnable_addr  = Enable;
    *(uint32_t *)h2p_lw_PickerPwmDutyCycle_addr = dutycycle;
    *(uint32_t *)h2p_lw_PickerPwmFrequency_addr = frequecy;
}

void BodyMotor(int direction,int Enable,int dutycycle,int frequecy)
{
    *(uint32_t *)h2p_lw_BodyDir_addr          = direction;
    *(uint32_t *)h2p_lw_BodyPwmEnable_addr  = Enable;
    *(uint32_t *)h2p_lw_BodyPwmDutyCycle_addr = dutycycle;
    *(uint32_t *)h2p_lw_BodyPwmFrequency_addr = frequecy;
}

void Fan1Motor(int direction,int Enable,int dutycycle,int frequecy)
{
    *(uint32_t *)h2p_lw_Fan1Dir_addr          = direction;
    *(uint32_t *)h2p_lw_Fan1PwmEnable_addr  = Enable;
    *(uint32_t *)h2p_lw_Fan1PwmDutyCycle_addr = dutycycle;
    *(uint32_t *)h2p_lw_Fan1PwmFrequency_addr = frequecy;
}

void Fan2Motor(int direction,int Enable,int dutycycle,int frequecy)
{
    *(uint32_t *)h2p_lw_Fan2Dir_addr          = direction;
    *(uint32_t *)h2p_lw_Fan2PwmEnable_addr  = Enable;
    *(uint32_t *)h2p_lw_Fan2PwmDutyCycle_addr = dutycycle;
    *(uint32_t *)h2p_lw_Fan2PwmFrequency_addr = frequecy;
}

int SendControlCMDToCar(int command_num,int speedLeft,int speedRight)
{
    switch(command_num)
	{
		//驱动电机动作
		case Stop: //Stop all motors 
        {
            LMotor(LMOTOR_GOAHEAD,TURN_OFF,DUTY_PULSE,TOTAL_PULSE);	
            RMotor(RMOTOR_GOAHEAD,TURN_OFF,DUTY_PULSE,TOTAL_PULSE);		
            PickerMotor(PICKER_UP,TURN_OFF,DUTY_PULSE,TOTAL_PULSE);
            BodyMotor(BODY_RAISE ,TURN_OFF,DUTY_PULSE,TOTAL_PULSE);	
            Fan1Motor(FAN1_PUFF  ,TURN_OFF,DUTY_PULSE,TOTAL_PULSE);
            Fan2Motor(FAN1_PUFF  ,TURN_OFF,DUTY_PULSE,TOTAL_PULSE);
            printf("Stop!\n");
            break;
        }             			
		case GoAhead://Go forward
        {
            if(speedLeft < 0 )
				speedLeft = 0;
			if(speedRight < 0 )
				speedRight = 0;	

            LMotor(LMOTOR_GOAHEAD,TURN_ON,speedLeft ,TOTAL_PULSE);	
            RMotor(RMOTOR_GOAHEAD,TURN_ON,speedRight,TOTAL_PULSE);
            printf("GoAhead!(%d,%d)\n",speedLeft,speedRight);	
            break;
        } 
		case GoBack://Go Back	
        {
            if(speedLeft < 0 )
				speedLeft = 0;
			if(speedRight < 0 )
				speedRight = 0;	
                         
            LMotor(LMOTOR_GOBACK,TURN_ON,speedLeft ,TOTAL_PULSE);	
            RMotor(RMOTOR_GOBACK,TURN_ON,speedRight,TOTAL_PULSE);
            printf("GoBack!(%d,%d)\n",speedLeft,speedRight);	           
            break;
        } 		
		case TurnRight://TurnRight	
        {
            if(speedLeft < 0 )
				speedLeft = 0;
			if(speedRight < 0 )
				speedRight = 0;	
                         
            LMotor(LMOTOR_GOBACK,TURN_ON,speedLeft ,TOTAL_PULSE);	
            RMotor(RMOTOR_GOAHEAD,TURN_ON,speedRight,TOTAL_PULSE);     
            printf("TurnRight!(%d,%d)\n",speedLeft,speedRight);                  
            break;
        }
		case TurnLeft://TurnLeft 	
        {
            if(speedLeft < 0 )
				speedLeft = 0;
			if(speedRight < 0 )
				speedRight = 0;	
                         
            LMotor(LMOTOR_GOAHEAD ,TURN_ON,speedLeft ,TOTAL_PULSE);	
            RMotor(RMOTOR_GOBACK,TURN_ON,speedRight,TOTAL_PULSE);
            printf("TurnLeft!(%d,%d)\n",speedLeft,speedRight);                           
            break;
        }		

		//捡球支架
		case TurnDownPickerBody://TurnDownPicker
        {
            if(*h2p_lw_FalingIn_addr==0)
            {
                LMotor(LMOTOR_GOAHEAD,TURN_OFF,speedLeft ,TOTAL_PULSE);	
                RMotor(RMOTOR_GOAHEAD,TURN_OFF,speedRight,TOTAL_PULSE);
                BodyMotor(BODY_FALLING ,TURN_ON,speedLeft,TOTAL_PULSE); 
                //将时间切片　以实现更小的时间检测
                usleep(UpDownDelayTime/6); 
                if(*h2p_lw_FalingIn_addr==0)
                {
                    usleep(UpDownDelayTime/6);
                    if(*h2p_lw_FalingIn_addr==0)
                    {
                        usleep(UpDownDelayTime/6);
                        if(*h2p_lw_FalingIn_addr==0) 
                        {
                            usleep(UpDownDelayTime/6);
                            if(*h2p_lw_FalingIn_addr==0) 
                            {
                                usleep(UpDownDelayTime/6);
                                if(*h2p_lw_FalingIn_addr==0)
                                {
                                    usleep(UpDownDelayTime/6);
                                }
                            }
                             
                        }
                                                  
                    }
                } 
            }
                 
            BodyMotor(BODY_RAISE ,TURN_OFF,speedLeft,TOTAL_PULSE);   
            printf("TurnDownPickerBody!(%d,%d)\n",speedLeft,speedRight);     
            break;
        }
		case TurnUpPickerBody://TurnUpPicker	 	
        {
            if(*h2p_lw_RaiseIn_addr==0)
            {
                LMotor(LMOTOR_GOAHEAD,TURN_OFF,speedLeft ,TOTAL_PULSE);	
                RMotor(RMOTOR_GOAHEAD,TURN_OFF,speedRight,TOTAL_PULSE);
                BodyMotor(BODY_RAISE ,TURN_ON,speedLeft,TOTAL_PULSE);
                //将时间切片　以实现更小的时间检测
                usleep(UpDownDelayTime/6); //--->200ms
                if(*h2p_lw_RaiseIn_addr==0)
                {
                    usleep(UpDownDelayTime/6);//--->400ms
                    if(*h2p_lw_RaiseIn_addr==0)
                    {
                        usleep(UpDownDelayTime/6);//--->600ms
                        if(*h2p_lw_RaiseIn_addr==0) 
                        {
                            usleep(UpDownDelayTime/6); //--->800ms
                            if(*h2p_lw_RaiseIn_addr==0)
                            {
                                usleep(UpDownDelayTime/6); //--->1000ms
                                if(*h2p_lw_RaiseIn_addr==0)
                                {
                                    usleep(UpDownDelayTime/6);
                                }
                            }
                        }
                                                 
                    }
                } 
            }
            BodyMotor(BODY_RAISE ,TURN_OFF,speedLeft,TOTAL_PULSE); 
            printf("TurnUpPickerBody!(%d,%d)\n",speedLeft,speedRight);                      
            break;
        }        
		case HandOnPickerBody://HandOnPicker	   		
        {
            BodyMotor(BODY_RAISE ,TURN_OFF,speedLeft,TOTAL_PULSE); 
            printf("HandOnPickerBody!\n");                        
            break;
        }

        //捡球飞轮
        case TurnUpFlyWheel://Picking Up 
        {
            PickerMotor(PICKER_UP,TURN_ON,speedLeft,TOTAL_PULSE); 
            printf("TurnUpFlyWheel!(%d,%d)\n",speedLeft,speedRight);               
            break;
        }
		case TurnDownFlyWheel://Picking Down 
        {
            PickerMotor(PICKER_DOWN,TURN_ON,DUTY_PULSE,TOTAL_PULSE); 
            printf("TurnDownFlyWheel!(%d,%d)\n",speedLeft,speedRight);            
            break;
        }
		case TurnOffFlyWheel://Stop Picking 
        {
            PickerMotor(PICKER_DOWN,TURN_OFF,DUTY_PULSE,TOTAL_PULSE); 
            printf("TurnOffFlyWheel!\n");           
            break;
        }        

		//风扇
		case TurnOnFan:	//Turn On Wind 	
        {
            Fan1Motor(FAN1_PUFF ,TURN_ON,speedLeft,TOTAL_PULSE);
            Fan2Motor(FAN2_PUFF ,TURN_ON,speedRight,TOTAL_PULSE);  
            printf("TurnOnFan!(%d,%d)\n",speedLeft,speedRight);           
            break;
        }	
		case TurnOffFan://Turn Off Wind	 	
        {
            Fan1Motor(FAN1_PUFF ,TURN_OFF,DUTY_PULSE,TOTAL_PULSE);
            Fan2Motor(FAN2_PUFF ,TURN_OFF,DUTY_PULSE,TOTAL_PULSE); 
            printf("TurnOffFan!\n");           
            break;
        }   			
		default:
		break;
	}
    return 0;
}

void TestDriver()
{
    SendControlCMDToCar(GoAhead,0.25*7199,0.25*7199);
    SendControlCMDToCar(TurnUpFlyWheel,0.10*7199,0);
    SendControlCMDToCar(TurnUpPickerBody,7199,0);
    SendControlCMDToCar(TurnOnFan,0.75*7199,0);
    usleep(500*1000);
    printf("\nSendControlCMDToCar(GoAhead\nSendControlCMDToCar(TurnUpFlyWheel\nSendControlCMDToCar(TurnUpPickerBody\nSendControlCMDToCar(TurnOnFan\n");
}
int TestDriverDirection()
{
    //
    switch (runOrStop())
    {
    case 0: 
        SendControlCMDToCar(GoAhead,0.15*7199,0.15*7199);
        break;
    case 1:
        SendControlCMDToCar(GoBack,0.15*7199,0.15*7199);
        break;
    case 2:
         SendControlCMDToCar(TurnLeft,0.15*7199,0.15*7199);
        break;
    case 3:
        SendControlCMDToCar(TurnRight,0.15*7199,0.15*7199);
        break;
    case 4:
        SendControlCMDToCar(TurnOnFan,0.3*7199,     0);//FAN1
        break;
    case 5:
        SendControlCMDToCar(TurnOnFan,     0,0.3*7199); //FAN2
        break;
    case 6:
        SendControlCMDToCar(TurnUpPickerBody,7199,0);
        break;
    case 7:
        SendControlCMDToCar(TurnDownPickerBody,7199,0);
        break;
    case 8:
        SendControlCMDToCar(TurnUpFlyWheel,0.12*7199,0);
        break;
    case 9:
        SendControlCMDToCar(TurnDownFlyWheel,0.1*7199,0);    
        break;
    case 15:
        SendControlCMDToCar(Stop,0,0);
        return 15;    
    default:
        break;
    }
    usleep(1000*1000);   

}


