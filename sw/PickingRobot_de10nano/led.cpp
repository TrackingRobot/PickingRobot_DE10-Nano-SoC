#include "led.h"

#define DEBUG 		1
#define DEBUG_DIR 	1


//用来设置方向的IO
extern unsigned long *h2p_lw_led_addr;//led灯的地址


int led()
{

	//static int blink_count=1;
	static int led_mask=0x0;
	static int led_direction=0;

	*(uint32_t *)h2p_lw_led_addr = led_mask;//点灯为正逻辑
	if(led_direction == 0)
	{
		led_mask = led_mask<<2;//左移,
		if(led_mask >= (0x01 << (LED_PIO_DATA_WIDTH - 1) ) )
			led_direction = 1;		
	}
	else
	{
		led_mask = led_mask >>2;
		if(led_mask <= 0x01)
		{
			led_direction = 0;
			//blink_count++;
		}
	}
	return 0;
}