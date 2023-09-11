#include "inputdetect.h"

#ifdef DEBUG_ALT_INTERRUPT
  #define dprintf printf
#else
  #define dprintf null_printf
#endif

//用来检测限位开关的传感器
extern unsigned long *h2p_lw_FalingIn_addr;	//falling_S_IN的地址;
extern unsigned long *h2p_lw_RaiseIn_addr;	//raise_S_IN的地址;
//用来管理程序的运行或暂停的IO
extern unsigned long *h2p_lw_StartOrPause_addr;
//用来管理程序终止或者运行的输入IO
extern unsigned long *h2p_lw_SwitchSW3_addr;

int raiseInput()
{
    return *(uint32_t *)h2p_lw_RaiseIn_addr;
}

int fallingInput()
{
    return *(uint32_t *)h2p_lw_FalingIn_addr;
}

int startOrPause()
{
    return *(uint32_t *)h2p_lw_StartOrPause_addr;
}

int runOrStop()
{
    return *(u_int32_t *)h2p_lw_SwitchSW3_addr;
}