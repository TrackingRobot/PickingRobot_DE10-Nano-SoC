#ifndef __INPUTDETECT_H
#define __INPUTDETECT_H

#include "stdio.h"
#include <inttypes.h>
#include "include/alt_interrupt.h"
#include "include/socal/socal.h"
#include "include/hwlib.h"
//#include "alt_printf.h"

#include "led.h"



int raiseInput();
int fallingInput();
int startOrPause();
int runOrStop();

#endif
