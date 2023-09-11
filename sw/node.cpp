/*
*  3iRoboticsLIDAR System II
*  Driver Interface
*
*  Copyright 2017 3iRobotics
*  All rights reserved.
*
*	Author: 3iRobotics, Data:2017-09-15
*
*/
#include "node.h"
#include "modify.h"


using namespace std;
using namespace everest::hwdrivers;

    
bool Is_Picker_Down_Flag=1;
bool Is_Use_WideRange=1;
bool Is_Miss_Boll_And_Wheel=0;

int PickingRadius=1500;

int main(int argc, char * argv[])
{
	int    opt_com_baudrate = 230400;
    string opt_com_path = "/dev/ttyAL0";
    struct timeval tb,te;
    //dingyiyige baningbianjing
    
    //用于程序自启动
    //daemon(0,0);
    //用于分割打印
    int i_count=0;
    printf("open uart 230400 completed\n");

    CSerialConnection serial_connect;
    C3iroboticsLidar  robotics_lidar;   

    //定义一个用来接收雷达数据的结构体数组,成员有Radius和Angles
    ScanPoints ScanPointsArry[A_CIRCLE]={0};
    
    //外设地址初始化   
    driverInit();
    printf("driverInit completed\n");
    // //开启芯片
    // turnOnTB6612FNG( TB6612ENABLE );//一直报段错误Segmentation
    printf("turnOnTB6612FNG completed !\n");
    //所有电机停止运行，以防误动作
    SendControlCMDToCar(Stop,0,0);

    //huoqu banjjing
    if(startOrPause()==1)//woziji SW
    {
        switch ( runOrStop() ) //banzide SW
        {
        case 8:
            PickingRadius = 1000;
            break;
        case 9:
            PickingRadius = 1500;
            break;
        case 10:
            PickingRadius = 2000;
            break;
        default:
            PickingRadius = 1200;
            break;
        }
    }
    
    //判断支架处于降下状态还是升起状态
    if( fallingInput()==1 )
        Is_Picker_Down_Flag=1;
    if( raiseInput()==1 )
        Is_Picker_Down_Flag=0;

    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());
    if(serial_connect.openSimple())
    {
        printf("[AuxCtrl] Open serail port sucessful!\n");
    }
    else
    {
        printf("[AuxCtrl] Open serail port %s failed! \n", opt_com_path.c_str());
        //所有电机停止运行，以防误动作
        SendControlCMDToCar(Stop,0,0);
        return -1;
    }

    printf("RobotLidar connected\n");

    robotics_lidar.initilize(&serial_connect);

	//recv and Analysis  thread
	bool ret = robotics_lidar.RecvAndAnalysisPthread(&robotics_lidar);
	if(ret)
		printf("RecvAndAnalysisPthread create success!\n");
	else
		printf("RecvAndAnalysisPthread create fail!\n");
    

	TLidarError retvalue;
	
    int FailTimes=5;
    #define STOPLIDAR 0

  	//start scan：AA 08 00 04 01 01 00 01 B9 00  
	#if !(STOPLIDAR)
	retvalue = robotics_lidar.setLidarWorkMode(HIGHSPEED_SCAN);//
	if(retvalue == EXECUTE_SUCCESS )
    {
        printf("High speed scan set successs!\n");	
    }

	else
    {
        i_count=0;
        //设定转速失败后,重新再试FailTimes次
        while( (FailTimes--) && (retvalue != EXECUTE_SUCCESS) )
        {
            //所有电机停止运行，以防误动作
            SendControlCMDToCar(Stop,0,0);

            retvalue = robotics_lidar.setLidarWorkMode(HIGHSPEED_SCAN);
            usleep(100*1000);
		    printf("%d High speed scan set fail! ...TLidarError=%d...\n",i_count++,retvalue);

        }
    }
	#endif
	
	//reset lidar：
	#if 0
	retvalue = robotics_lidar.setLidarWorkMode(LIDAR_RESET);
	if(retvalue == EXECUTE_SUCCESS )
		printf("LIDAR reset successs!\n");
	else
		printf("LIDAR reset fail! ...TLidarError=%d...\n",retvalue);
	#endif
	//stop scan: AA 08 00 04 01 01 00 00 B8 00   return: AA 08 00 04 41 01 00 00 F8 00 
	
    
    #if 0
	retvalue = robotics_lidar.setLidarWorkMode(IDLE_MODE);
	if(retvalue == EXECUTE_SUCCESS )
		printf("stop scan set successs!\n");
	else
		printf("stop scan set fail! ...TLidarError=%d...\n",retvalue);
	#endif
	//set lidar rotationSpeed: AA 09 00 04 04 02 00 0B 00 C8 00 return:AA 08 00 04 44 01 00 00 FB 00
	#if 0
	retvalue = robotics_lidar.setLidarRotationlSpeed(11);
	if(retvalue == EXECUTE_SUCCESS )
		printf("RotationlSpeed set successs!\n");
	else
		printf("RotationlSpeed set fail! ...TLidarError=%d...\n",retvalue);
	#endif

    
    #if !(STOPLIDAR)
    //开风扇,为简化控制默认开启风扇
    SendControlCMDToCar(TurnOnFan,0.2*SPEED_FULL,0.2*SPEED_FULL);

    //开启捡球飞轮,为简化控制过程默认进入程序就开启捡球飞轮
    SendControlCMDToCar(TurnUpFlyWheel,0.12*7199,0);	
    //升高支架
    SendControlCMDToCar(TurnUpPickerBody,SPEED_FULL,0);
    //usleep(UpDownDelayTime);//上升\下降期间测量    
    Is_Picker_Down_Flag=0;
    Is_Use_WideRange=1; 

    // //放下支架
    // SendControlCMDToCar(TurnDownPicker,95,0);
    // Is_Picker_Down_Flag=1;
    // Is_Use_WideRange=0;
	
	TLidarGrabResult    result;
	CLidarDynamicScan   lidar_scan;
	std::vector<RslidarDataComplete> send_lidar_scan_data;
	int lidar_scan_size;
	int i,size = 0;
    
	RslidarDataComplete one_lidar_data;
   	while (1)
    { 	
         
        result = robotics_lidar.m_dynamic_scan.getGrabResult();
        //result = LIDAR_GRAB_SUCESS;
        switch(result)
        {
            case LIDAR_GRAB_ING:
            {
                break;
            }
            //每1600次循环中有一次能进去该case
            case LIDAR_GRAB_SUCESS:
            {
                // gettimeofday(&tb,NULL);
                // printf("begin:%ld\n",tb.tv_sec*1000000 + tb.tv_usec);
                //printf("begin:%ld\n",tv.tv_sec*1000000 + tv.tv_usec);  //微秒
				robotics_lidar.m_dynamic_scan.resetGrabResult();
					
                lidar_scan = robotics_lidar.getLidarDynamicScan();
                lidar_scan_size = lidar_scan.getSize();
                send_lidar_scan_data.resize(lidar_scan_size);  
                #if DEBUG_AllRangePoints
                printf("-----------------%4d\n",lidar_scan_size);
                #endif                     
				//one circle: angle and  distance
                for( i = 0; i < lidar_scan_size; i++)
                {            
                    one_lidar_data.angle = lidar_scan.m_angle[i];
                    one_lidar_data.distance = lidar_scan.m_distance[i];
                    send_lidar_scan_data[i] = one_lidar_data;
                    
                    //20190224_将采集到的数据存入数组		   
			        ScanPointsArry[i].Radius=one_lidar_data.distance*1000;
                    ScanPointsArry[i].Angles=one_lidar_data.angle;

                    #if DEBUG_AllRangePoints
                    usleep(30*1000);
                    printf("(%4.0f,%3.3f) ",ScanPointsArry[i].Radius,ScanPointsArry[i].Angles);
                    if(i%12==0)
                        printf("\n");
                    if(i==lidar_scan_size)
                    printf("-----------------%4d\n",lidar_scan_size);
                    #endif
                   
                }

				//one circle:Total number of points
                //printf("Lidar count %d!\n", lidar_scan_size);
                #if !(DEBUG_AllRangePoints)
               
                ModifyPoints( ScanPointsArry,lidar_scan_size );	
                // gettimeofday(&te,NULL);
                // printf("end:%ld, %ld\n",te.tv_sec*1000000 + te.tv_usec,
                //    te.tv_sec*1000000 + te.tv_usec - tb.tv_sec*1000000 - tb.tv_usec);  //微秒
                #endif
                //LED跑马灯表示正常工作
                led();

                break;
            }
            case LIDAR_GRAB_ERRO:
            {
                printf("LIDAR_GRAB_ERRO\n");
                SendControlCMDToCar(Stop,0,0);
                break;
            }
            case LIDAR_GRAB_ELSE:
            {
                printf("[Main] LIDAR_GRAB_ELSE!\n");
                SendControlCMDToCar(Stop,0,0);
                break;
            }
        }
       
        usleep(10);

        //当滑动开关拨至Pause(低电平),程序暂定运行	
		//while(*h2p_lw_StartOrPause_addr==0)
        while(startOrPause()==0)
		{
			SendControlCMDToCar(Stop,0,0);
            printf("pause^^^^^^^^\n");
            //当开关SW3拨至1[1***]则停止程序
            if( TestDriverDirection() == 15)
            {
                SendControlCMDToCar(Stop,0,0);
                retvalue = robotics_lidar.setLidarWorkMode(IDLE_MODE);
                if(retvalue == EXECUTE_SUCCESS )
                    printf("manual stop scan set successs!\n");
                return 0;
            }
		}    
    }
	#endif   //end stoplidar
    return 0;
}

/*动态扫描的数据
AA 6D 00 04 14 66 00 E4 03 78 69 00 00 7E 2C 68 2C 7E 2C 86 2C 79 2B 09 2B 8A 2B 73 25 00 00 00 00 42 26 49 26 00 00 1A 27 AD 26 00 00 00 00 00 00 00 00 00 00 00 00 00 00 9B 63 9B 63 F1 65 CC 63 EE 29 3B 2C 18 29 20 29 9F 26 56 26 B4 26 64 19 3F 19 6A 19 BE 19 21 1A 3C 1A 8A 1A BF 1A F9 1A 3A 1B B6 1B 0D 1C AE 1C A7 1D 1C 1F 13 1B 
*/








