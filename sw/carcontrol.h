#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H

#include "node.h"

// enum CONTROL_CMD
// {

// 		Stop=0,           //Car stop
//     	TurnRight=1,
// 		TurnLeft,       //TurnLeft	
// 		GoAhead,        //Go forward 
// 		GoBack,         //Go Back 

// 		//捡球装置
// 		TurnDownPickerBody, //TurnDownPicker	
// 		HandOnPickerBody,   //HandOnPicker			
// 		TurnUpPickerBody,   //TurnUpPicker 
// 		//捡球飞轮
// 		TurnUpFlyWheel,
//         TurnOffFlyWheel,
// 		TurnDownFlyWheel,
// 		//风扇
// 		TurnOnFan,      //Turn On Wind		
// 		TurnOffFan      //Turn Off Wind			
		
// };


int CarActControl(float &radius,float &angle);

extern int SpeedP,SpeedI,SpeedD; 
extern int ThisAngleBias,LastAngleBias,PreAngleBias;
extern int PwmMotorTmep,PwmMotor;



#endif
