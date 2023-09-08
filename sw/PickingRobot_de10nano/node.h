#ifndef __NODE_H
#define __NODE_H

#include <sys/time.h>
#include "stdlib.h"
#include "include/alt_hwlibs_ver.h"
#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"

//#include "modify.h"
#include "uart.h"
#include "carcontrol.h"
#include "led.h"
#include "inputdetect.h"

#include "videoCapture.h"

#define MAXPOINT (8000)
#define A_CIRCLE (1600)  //高速扫描模式，转速为5r/s，每圈的点数为1600，实际情况一般为1520个点

#define SPEED_STOP          200     //电机的停止速度
#define SPEED_FULL          7199    //电机电机全速
#define SPEED_SLOWMOVE      1200    //点击缓慢移动速度
#define UpDownDelayTime     1200*1000    //捡球支架上升\下降所需要的时间(判断条件改为　while(限位开关为低电平))
#define PickerForce         800*1000//捡球轮的力度增加参数，根据实验所得

//TB6612芯片开启标志
#define TB6612ENABLE    0
#define TB6612DISNABLE  1

#define DEG2RAD(x) ((x)*M_PI/180.)
//用于调试输出坐标
#define DEBUG_AllRangePoints 0        //值为1时将打印出360度所有的点

typedef struct _rslidar_data
{
    _rslidar_data()
    {
        //signal = 0;
        angle = 0.0;
        distance = 0.0;
    }
    //uint8_t signal;
    float   angle;
    float   distance;
}RslidarDataComplete;



typedef struct 
{
    float Radius;
    float Angles;
}ScanPoints;

extern bool Is_Picker_Down_Flag;
extern bool Is_Use_WideRange;
extern bool Is_Miss_Boll_And_Wheel;


#endif
