#ifndef __MODIFY_H
#define __MODIFY_H

#include "node.h"

#define FRONT_ANGLE      (75.15) 	//定义正前方角度
#define DEVIATION_ANGLE  2  	    //定义角度偏差为2度，球在角度偏差范围内都能捡到
#define FRONT_DISTANCE   300	    //该距离下捡球轮恰好能捡到球
#define DEVIATION_RADIUS 50 	    //考虑测量误差

#define CHANGE_GRAD 			70		//两个点之间最大变化值为70mm
#define LIDAR_SCAN_START_ANGLE 	22.5	//limit  20.0,捡球飞轮升起后可视范围起始角
#define LIDAR_SCAN_END_ANGLE   	129.5	//limit 130.0,捡球飞轮升起后可视范围终止角
#define MAX_RADIUS  			1500	//最大量程3500
#define MAX_SHUTTLECOCK 		50		//可视范围最多有n个球

#define MIDDLE_PICK_START_ANGLE 72.2	//limit74.38,捡球飞轮降下后可视范围起始角
#define MIDDLE_PICK_END_ANGLE 	78.4	//limit79.89,捡球飞轮降下后可视范围终止角

#define LEFT_PICKER_START_ANGLE (58.454+1)  	//雷达测到的左边捡球飞轮的起始角度
#define LEFT_PICKER_END_ANGLE   (70.979+1)  	//雷达测到的左边捡球飞轮的终止角度
#define LEFT_PICKER_MIN_RADIUS  245//275         //左边捡球轮距离最小值
#define LEFT_PICKER_MAX_RADIUS  300         //左边捡球轮距离最大值

#define RIGHT_PICKER_START_ANGLE (79.330-1)   //雷达测到的左边捡球飞轮的起始角度
#define RIGHT_PICKER_END_ANGLE   (92.609-0) 	//雷达测到的左边捡球飞轮的终止角度
#define RIGHT_PICKER_MIN_RADIUS  240//279        //右边捡球轮距离最小值
#define RIGHT_PICKER_MAX_RADIUS  299        //右边捡球轮距离最大值

#define MIN_LENGTH_WHEEL		 20//雷达测到的左边捡球飞轮的最小长度
#define MAX_LENGTH_WHEEL		 90//雷达测到的左边捡球飞轮的最达长度

#define MAXLENTH 95          //羽毛球最大长度
#define MINLENTH 25          //羽毛球最小长度

#ifndef MAXPOINT
#define MAXPOINT (8000)
#endif

#ifndef A_CIRCLE
#define A_CIRCLE (1600)  //高速扫描模式，转速为5r/s，每圈的点数为1600，实际情况一般为1520个点
#endif

extern ScanPoints ScanPointsArry[A_CIRCLE];



int ModifyPoints(ScanPoints (&StructArry)[A_CIRCLE],int count);

#endif
