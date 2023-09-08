#include "stdio.h"
#include "string.h"
#include "math.h"
#include <unistd.h>
#include "uart.h"
#include "modify.h"
#include "node.h"
#include "carcontrol.h"

//打印调试
#define DEBUG_PROCESS 					0 //打印程序执行到哪一步了
#define DEBUG_CordinationOfBadminton 	1 //值为1时用于打印距离最近羽毛球坐标
#define DEBUG_MissUnderstand 			1 //值为1时打印 "误判捡球轮为球" 信息
#define DEBUG_Rotaion					1 //值为0时只有转向动作,没有前进后退动作
#define DEBUG_Chulihoudezuobiao         0 //计算的羽毛球坐标

#define SuitablePickingDistance 		480//捡球准备距离,当羽毛球距离激光雷达为该距离时要将平台对正羽毛球然后直行捡球
#define AttenuationFactor				0.5//左右轮的速度经过衰减后才能加在直行速度中,否则出现蛇形走位的情况
int SpeedP =2,SpeedI =2,SpeedD =2; 
int ThisAngleBias =0,LastAngleBias =0,PreAngleBias =0;
int PwmMotorTmep =0,PwmMotor =0;

bool  IS_BALL_IN_MIDDLE=0;


int CarActControl(float &radius,float &angle)
{
	float angle_bias,radius_bias;
	bool  Is_A_Ball_In_Middle_Radius=0,TurnLeftFlag=0,TurnRightFlag=0;

	bool  left_pick_ball_condition=0,right_pick_ball_condition=0;//左右参考依据是雷达的俯视方向
		
	static int miss_time=0,radius_zero_times=0;
	int speed=0,speedLeft=0,speedRight=0;
	static int TempRadius[6],TempAngle[6],TempRadiusCount=0;
	int i,j,SimilarRadiusCount[6]={0},TempCase,TempSwapRadius,TempSwapAngle;
	int static lastAngleOfShuttlecock=angle;
	static int BallInMiddleTimes=0;
	float fanSpeedProportion=0.2;
	//
	// if(radius==0 || angle==0)
	// 	return 0;
	//Is_Picker_Down_Flag = !(Is_Use_WideRange);
	// ThisAngleBias = int(angle  - FRONT_ANGLE);	
	// ThisAngleBias = fabs(ThisAngleBias);
	//angle_bias =angle  - FRONT_ANGLE;
	//**********************过滤抖动的点*****************************//
	//角度抖动过滤
	//如果本次给出的羽毛球角度值和上次的羽毛球角度差别太大(大于20度),则返回再计算
	if(fabs(lastAngleOfShuttlecock-angle)>20)
	{	    
		lastAngleOfShuttlecock=angle;
		return 0;
	}
    lastAngleOfShuttlecock=angle;
	//距离抖动过滤
	//此部分的代码已被删除,若需要添加,移步目录:
	//   /home/lyb/lidar/de1_bak/integration_de1_20191019


	angle_bias =angle  - FRONT_ANGLE;
	radius_bias=radius - FRONT_DISTANCE;
	#if DEBUG_CordinationOfBadminton
	printf("(%4.0f,%3.2f),%3.2f\n",radius,angle,angle_bias);
	#endif
	//判断是否将捡球轮误认为是羽毛球
	//判断依据是支架下降后捡球轮和激光雷达之间角度和距离关系

	if(Is_Picker_Down_Flag==1)
	{
		// //误判左轮为球的情况
		// left_pick_ball_condition =(radius <= LEFT_PICKER_MAX_RADIUS && \
		// 						   radius >= LEFT_PICKER_MIN_RADIUS && \
		// 						   angle  >= LEFT_PICKER_START_ANGLE && \
		// 						   angle  <= LEFT_PICKER_END_ANGLE);
		// //误判右轮为球的情况						   
		// right_pick_ball_condition=(radius >= RIGHT_PICKER_MIN_RADIUS && \
		// 						   radius <= RIGHT_PICKER_MAX_RADIUS && \
		// 						   angle  >= RIGHT_PICKER_START_ANGLE && \
		// 						   angle  <= RIGHT_PICKER_END_ANGLE);

		//误判左轮为球的情况
		left_pick_ball_condition =(angle  >= LEFT_PICKER_START_ANGLE && \
								   angle  <= LEFT_PICKER_END_ANGLE);
		//误判右轮为球的情况						   
		right_pick_ball_condition=(angle  >= RIGHT_PICKER_START_ANGLE && \
								   angle  <= RIGHT_PICKER_END_ANGLE);
	}

	
	if(left_pick_ball_condition||right_pick_ball_condition)
		{
			Is_Miss_Boll_And_Wheel=1;			
			miss_time++;
			#if DEBUG_MissUnderstand
			printf("misunderstand\n");
			#endif
		}
	else
		{
			Is_Miss_Boll_And_Wheel=0;
			miss_time=0;
		}


	//如果(5圈/S)连续误将捡球轮认为是羽毛球8次,则停车并升起捡球轮支架
	//1秒5圈,也就是1秒误解5次
	#if DEBUG_PROCESS
	printf("miss_time==%d\n",miss_time);
	#endif

	if(miss_time>=5 || radius_zero_times>=5)
	{
		//miss_time>=8指的是将捡球轮误判为羽毛球的次数超过8次,
		//正常情况下,捡球轮支架下降后,雷达只扫描小范围,而不会出现误判捡球轮为羽毛球的情况
		if(Is_Picker_Down_Flag==1)
		{
			SendControlCMDToCar(GoAhead,SPEED_STOP,SPEED_STOP);
			SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL,0);
			//usleep(UpDownDelayTime);//上升\下降期间测量
			Is_Picker_Down_Flag=0;
			#if DEBUG_PROCESS
			printf("(miss_time>=5 || radius_zero_times>=5\n");
			#endif			
		}

		SendControlCMDToCar(GoBack,SPEED_STOP+100,SPEED_STOP+100);		
		
		radius_zero_times=0;
		miss_time=0;	
	}	

	//*******************************************计算转向速度*********************************************//
	// 调整角度,使得捡球机正对羽毛球//
	// 如果偏移,则进行左转或者右转	
	if( (fabs(angle_bias) > DEVIATION_ANGLE))//使用while程序就会一直卡在循环,接收不到最新数据
	{
		int static lastAnleBiase=0;
		int static CantTurnTimes=0;
		
		//方向偏移,支架升起才调整方向
		if( Is_Picker_Down_Flag==0 && (Is_Miss_Boll_And_Wheel==0) )
		{
			IS_BALL_IN_MIDDLE=0;	
			#define maxTurnSpeed (0.4*SPEED_FULL)	
			#define minTurnSpeed 800//860
			//SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL);
			//如果球偏右，则车右转
			if(angle_bias > (DEVIATION_ANGLE) )
				//右转,speedLeft>0\speedRight<0
				{
					//数值转换成字符  
					//sprintf(AngleCharBuf,"%3.3f",point_angle[0]);
					//sprintf(RadiusCharBuf,"%4.2f",point_radius[0]);	
					speed=fabs(angle_bias)/0.5+minTurnSpeed;

					//speed(5度)==800,speed(120度)==1020	

					//如果上次的偏移角度和本次便宜角度一样,说明小车没转动,这时需要加大速度让其转起来
					if( lastAnleBiase - fabs(angle_bias) <  0.1 ) 
						CantTurnTimes++;
					else
					{
						CantTurnTimes=0;
					}

					if(CantTurnTimes>=3)
					{
						speed+= (0.25*speed*CantTurnTimes); 
					}
					if(speed>maxTurnSpeed)
						speed=maxTurnSpeed;


					speedLeft = speed;
					speedRight=-speed;

					TurnRightFlag=1;
					//SendControlCMDToCar(TurnRight,speed);//"TurnRight"
					//为了解决转向过冲问题,细分转向时间,每次转向100ms
					// usleep(angle_bias*10*1000);
					// SendControlCMDToCar(TurnRight,SPEED_STOP+150);
						
				}
			//如果球偏左，则车左转
			if(angle_bias < -(DEVIATION_ANGLE ) )
				//左转,speedLeft<0\speedRight>0
				{
					speed=fabs(angle_bias)/0.5+minTurnSpeed;//speed(5度)==800,speed(120度)==1020										
					//如果上次的偏移角度和本次偏移角度一样,说明小车没转动,这时需要加大速度让其转起来
					if(lastAnleBiase - fabs(angle_bias) < 0.1 )
						CantTurnTimes++;
					else
					{
						CantTurnTimes=0;
					}

					if(CantTurnTimes>=2)
					{
						speed+= (0.25*speed*CantTurnTimes); 
					}
					if(speed>maxTurnSpeed)
						speed=maxTurnSpeed;

					speedLeft =-speed;
					speedRight= speed;

					TurnLeftFlag=1;	
					//SendControlCMDToCar(TurnLeft,speed);
					//为了解决转向过冲问题,细分转向时间,每次转向100ms
					// usleep(angle_bias*10*1000);
					// SendControlCMDToCar(TurnLeft,SPEED_STOP+100);
						

				}
			//保存本次角度偏差,作为下次运算的参考值
			lastAnleBiase=fabs(angle_bias);	
		}
		//方向偏移,支架降下时,可能是误判.
		// else
		// {
		// 	SendControlCMDToCar(GoAhead,SPEED_STOP-500,SPEED_STOP-500);/* code */
		// 	SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL,0);
		// 	usleep(UpDownDelayTime);//上升\下降期间测量
		// 	Is_Picker_Down_Flag=0;
		// }			
		IS_BALL_IN_MIDDLE=0;
		fanSpeedProportion = (fabs(angle_bias)-(DEVIATION_ANGLE))\
								/(2) * 0.8;
		if(fanSpeedProportion > 0.8)
			fanSpeedProportion=0.8;
		if(fanSpeedProportion< 0.1)
			fanSpeedProportion=0.1;	
	}
	//如果球没有偏移，则停止转向
	else
	{
		IS_BALL_IN_MIDDLE=1;
		TurnLeftFlag=0;
		TurnRightFlag=0;
		BallInMiddleTimes++;
		SendControlCMDToCar(TurnOnFan,0.1*SPEED_FULL,0.1*SPEED_FULL);
		// SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL);	
		// Is_Picker_Down_Flag=0;
		SendControlCMDToCar(GoAhead,SPEED_STOP-500,SPEED_STOP-500);
		#if DEBUG_PROCESS
		printf("IS_BALL_IN_MIDDLE=1;\n");
		#endif
		speedLeft = 0;
		speedRight= 0;		
		// SendControlCMDToCar(TurnLeft,10);	
		// SendControlCMDToCar(TurnRight,10);			
	}
#if DEBUG_Rotaion
	//********************************************行走与捡球***********************************************//
	//若距离距离合适:   转向对正羽毛球    ---->直行捡球
	//若距离距离不合适: 走曲线至合适的距离 ---->曲线行走
	static int GoBackTimes=0;
	if(radius<=350)//以免距离==300时 捡球装置不停滴上下摆动
	{
		
		SendControlCMDToCar(GoBack,1.5*1.2*SPEED_SLOWMOVE,1.5*1.2*SPEED_SLOWMOVE);
		GoBackTimes++;
		if(GoBackTimes>=5)
			SendControlCMDToCar(GoBack,0.8*SPEED_STOP,2*1.3*SPEED_SLOWMOVE);
	}
	//300~~~~~~~SuitablePickingDistance为捡球距离
	else if(radius >340 && radius < SuitablePickingDistance)
	{
		GoBackTimes=0;
		//********************************************转向*********************************************
		if( (!IS_BALL_IN_MIDDLE) && (Is_Miss_Boll_And_Wheel==0) )
		{
			if(TurnLeftFlag==1)
			{
				SendControlCMDToCar(TurnRight,0.85*speed,0.9*speed);
				SendControlCMDToCar(TurnOnFan,0.1*SPEED_FULL,fanSpeedProportion*SPEED_FULL);
			}
				
			if(TurnRightFlag==1)
			{
				SendControlCMDToCar(TurnLeft ,0.9*speed,0.85*speed);				
				SendControlCMDToCar(TurnOnFan,fanSpeedProportion*SPEED_FULL,0.1*SPEED_FULL);
			}
				

			//printf("TurnLeft or TurnRight\n");	

			return 0;
		}
		//捡球
		else
		{
			if(Is_Miss_Boll_And_Wheel==0)
			{
				
				if(Is_Picker_Down_Flag==0 && IS_BALL_IN_MIDDLE && BallInMiddleTimes>=1)
				{
					SendControlCMDToCar(GoBack,0.5*SPEED_STOP,0.5*SPEED_SLOWMOVE);
					SendControlCMDToCar(TurnOnFan,0.4*SPEED_FULL,0.4*SPEED_FULL);
					SendControlCMDToCar(TurnDownPickerBody,0.95*SPEED_FULL,0);
					//usleep(1.2*UpDownDelayTime);
					Is_Picker_Down_Flag=1;
				}
				/************Picking**************/
				if(BallInMiddleTimes>=3)
				{
					speed=(radius-290)/0.3738+1370+300;//speed(290)==1370,speed(370)==1584		
					SendControlCMDToCar(GoAhead,speed,speed);
					SendControlCMDToCar(TurnUpFlyWheel,1279-450,0);
					
					usleep( (0.7+(radius-290)/(SuitablePickingDistance-290))*PickerForce );//上升\下降期间测量 
					SendControlCMDToCar(TurnUpFlyWheel,1279-450,0);
					SendControlCMDToCar(TurnOnFan,0.1*SPEED_FULL,0.1*SPEED_FULL);
					BallInMiddleTimes=0;
				}

				#if DEBUG_PROCESS
				printf("(radius>=290) && (radius<370)\n");
				#endif
				return 0;	
			}
				
		}

	}
	//********************************************行走
	else
	{
		
		GoBackTimes=0;
		SendControlCMDToCar(GoAhead,SPEED_STOP,SPEED_STOP);
		SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL,0);
		Is_Picker_Down_Flag=0;
		//大于SuitablePickingDistance则往前走
		//SuitablePickingDistance-480mm,是球可捡,下降支架同时前进
		if(radius>=SuitablePickingDistance && radius<(SuitablePickingDistance+250) && (Is_Miss_Boll_And_Wheel==0))
		{
	
			// if(Is_Picker_Down_Flag==0 && IS_BALL_IN_MIDDLE)
			// {
			// 	SendControlCMDToCar(TurnDownPickerBody,SPEED_FULL,0);
			// 	SendControlCMDToCar(TurnOnFan,0.3*SPEED_FULL,0.3*SPEED_FULL);
			// 	//usleep(UpDownDelayTime);//上升\下降期间测量
			// 	Is_Picker_Down_Flag=1;				
			// }
			speed=(radius-500)/1.0309+1584-800;//speed(SuitablePickingDistance)==0.18,speed(~ +200)==0.2474
			speedLeft *=AttenuationFactor;
			speedLeft += (1.15*speed);
			if(speedLeft >= 0.4*SPEED_FULL)
				speedLeft=0.4*SPEED_FULL;

			speedRight*=AttenuationFactor;
			speedRight+= (1.15*speed);
			if(speedRight >= 0.4*speed)
				speedRight=0.4*SPEED_FULL;

			SendControlCMDToCar(GoAhead,speedLeft,speedRight);
			radius_zero_times=0;

			#if DEBUG_PROCESS
			printf("radius>=370 && radius<520\n");
			#endif
			return 0;			
		}
			
		//430-MAX_DISTENCE,是球可捡,前进并升起支架
		else if(radius>=SuitablePickingDistance+250 && radius<SuitablePickingDistance+1000 &&(Is_Miss_Boll_And_Wheel==0))//该距离下不会出现误判,但为逻辑清楚加上了是否误判
		{

			if(Is_Picker_Down_Flag==1)
			{
				SendControlCMDToCar(GoAhead,SPEED_STOP-600,SPEED_STOP);
				SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL,0);
				//usleep(UpDownDelayTime);///上升\下降期间测量
				Is_Picker_Down_Flag=0;
			}

			speed=(radius-500)/2.6455+1781;//speed(500)==0.2474,speed(1500)==0.3  2160				
			speedLeft *=(1*AttenuationFactor);
			speedLeft += (1.6*speed);
			speedRight*=(1*AttenuationFactor);
			speedRight+= (1.6*speed);
			SendControlCMDToCar(GoAhead,speedLeft,speedRight);	
			SendControlCMDToCar(TurnOnFan,0.1*SPEED_FULL,0.1*SPEED_FULL);
			radius_zero_times=0;

			#if DEBUG_PROCESS
			printf("radius>=520 && radius<880\n");
			#endif
			return 0;		
		}

		else if(radius>=SuitablePickingDistance+1000 && radius<SuitablePickingDistance+2000 &&(Is_Miss_Boll_And_Wheel==0))
		{

			if(Is_Picker_Down_Flag==1)
			{
				SendControlCMDToCar(GoAhead,SPEED_STOP,SPEED_STOP);
				SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL,0);
				//usleep(UpDownDelayTime);//上升\下降期间测量
				Is_Picker_Down_Flag=0;
			}

			speed=(radius-880)/1.7811+2160-1200-300;//speed(880)==0.3,speed(1500)==0.35  2508							
			speedLeft *=(0.5*AttenuationFactor);
			speedLeft += (1.5*speed);
			speedRight*=(0.5*AttenuationFactor);
			speedRight+= (1.5*speed);
			SendControlCMDToCar(GoAhead,speedLeft,speedRight);	
			radius_zero_times=0;
			SendControlCMDToCar(TurnOnFan,0.1*SPEED_FULL,0.1*SPEED_FULL);
			#if DEBUG_PROCESS
			printf("radius>=880 && radius<1500\n");
			#endif
			return 0;						
		}

		//是球可捡,前进并升起支架
		else if(radius>=2000)
		{
			if(Is_Picker_Down_Flag==1)
			{
				SendControlCMDToCar(GoAhead,SPEED_STOP,SPEED_STOP);
				SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL,0);
				//usleep(UpDownDelayTime);//上升\下降期间测量
				Is_Picker_Down_Flag=0;
			}
			speed=(radius-1500)/2.985;//speed(1500)==0.3,speed(3000)==0.4  		2880
			speedLeft *=AttenuationFactor;
			speedLeft += (1.5*speed);
			speedRight*=AttenuationFactor;
			speedRight+= (1.5*speed);
			SendControlCMDToCar(GoAhead,speedLeft,speedRight);
			SendControlCMDToCar(TurnOnFan,0.1*SPEED_FULL,0.1*SPEED_FULL);
			radius_zero_times=0;

			#if DEBUG_PROCESS
			printf("radius>=1500\n");
			#endif
			return 0;						
		}
		
		#if DEBUG_PROCESS
		printf("Is_Use_WideRange ->%d\nIs_Miss_Boll_And_Wheel ->%d\n",Is_Use_WideRange);
		#endif		
	
	}
	
		#endif
	// //球在中间时才捡
	// if(IS_BALL_IN_MIDDLE==1 && Is_Miss_Boll_And_Wheel==0)
	// {
	// 	int static lastRadius=0;
	// 	int static CantHeadTimes=0;


	// 	//球不在中间,但是没有误判就要升起支架
	// 	if( IS_BALL_IN_MIDDLE==0 && Is_Miss_Boll_And_Wheel==0)
	// 	{
	// 		//SendControlCMDToCar(GoAhead,10);
	// 		if(Is_Picker_Down_Flag==1)
	// 		{
	// 			SendControlCMDToCar(GoAhead,SPEED_STOP,SPEED_STOP);
	// 			SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL,0);
	// 			//usleep(UpDownDelayTime);//上升\下降期间测量
	// 			Is_Picker_Down_Flag=0;	
	// 		}

	// 		#if DEBUG_PROCESS
	// 		printf("IS_BALL_IN_MIDDLE==0 && Is_Miss_Boll_And_Wheel==0\n");
	// 		#endif		

	// 	}
	// 	//球不在中间,但是误判
	// 	//球在中间,但是误判
	// 	else
	// 	{
	// 		;
	// 	}	
	// 	
	// 	return 0;   
	// }
}