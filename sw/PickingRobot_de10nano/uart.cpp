#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "carcontrol.h"
#include "node.h"
#include "uart.h"


#define DATAPACK_LENGTH  6//数据包长度

#define DEBUG_Command    0//用于打印控制小车的命令
//打开串口

int open_port(void)

{

	int fd;
		

	fd=open("/dev/ttyAL1",O_RDWR | O_NOCTTY | O_NONBLOCK);
	//printf("fd=%d\n",fd);	

	if(fd==-1)

	{
		//perror("Can't Open SerialPort");

		//printf("fopen error \n");
		usleep(50*1000);//original time is 500*1000
	}

	return fd;
}


int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop) 

{ 
     struct termios newtio,oldtio; 
    /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/ 
    memset(&newtio,0,sizeof(termios));
    memset(&oldtio,0,sizeof(termios));  


    if( tcgetattr( fd,&oldtio)  !=  0) 
    {  
    //perror("SetupSerial 1");
    //printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
    printf("error 2\n");
    usleep(50*1000);//original time is 500*1000
    return -1; 
    } 

    bzero( &newtio, sizeof( newtio ) ); 

/*步骤一，设置字符大小*/ 

    newtio.c_cflag  |=  CLOCAL | CREAD;  
    newtio.c_cflag &= ~CSIZE;  

/*设置停止位*/ 

    switch( nBits ) 
     { 
     case 7: 
      newtio.c_cflag |= CS7; 
      break; 

     case 8: 
      newtio.c_cflag |= CS8; 
      break; 

     } 

/*设置奇偶校验位*/ 

     switch( nEvent ) 
     { 
     case 'o':
     case 'O': //奇数 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag |= PARODD; 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      break; 

     case 'e':
     case 'E': //偶数 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag &= ~PARODD; 
      break;

	 case 'n':
     case 'N':  //无奇偶校验位 
     newtio.c_cflag &= ~PARENB; 
     break;

     default:
      break;
     } 

     /*设置波特率*/ 

switch( nSpeed ) 
     { 
     case 2400: 
      cfsetispeed(&newtio, B2400); 
      cfsetospeed(&newtio, B2400); 
      break; 

     case 4800: 
      cfsetispeed(&newtio, B4800); 
      cfsetospeed(&newtio, B4800); 
      break; 

     case 9600: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
      break; 

     case 115200: 
      cfsetispeed(&newtio, B115200); 
      cfsetospeed(&newtio, B115200); 
      break; 

     case 460800: 
      cfsetispeed(&newtio, B460800); 
      cfsetospeed(&newtio, B460800); 
      break; 

     default: 

      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
     break; 

     } 

/*设置停止位*/ 

     if( nStop == 1 ) 
      newtio.c_cflag &=  ~CSTOPB; 
     else if ( nStop == 2 ) 
      newtio.c_cflag |=  CSTOPB; 

/*设置等待时间和最小接收字符*/ 

     newtio.c_cc[VTIME]  = 0; 
     newtio.c_cc[VMIN] = 0; 

/*处理未接收字符*/ 
     tcflush(fd,TCIFLUSH); 
/*激活新配置*/ 

if((tcsetattr(fd,TCSANOW,&newtio))!=0) 
     { 
      //perror("com set error"); 
      printf("com set error\n");
      return -1; 
     } 

     //printf("set done!\n"); 
     return 0; 
} 



int SendControlCMDToCarByUart(int command_num,int speedLeft,int speedRight)

{

	static int fd=open_port();
     int i,count;
	int nSpeed=9600,nBits=8,nStop=1;
	char nEvent='N';
     char *buf,*buf1,*buf2,speed_buf[3]={0};
  

     //fd=open_port();
     set_opt(fd,nSpeed, nBits, nEvent, nStop);
	buf =(char *)malloc(sizeof(char)*16);	
	buf1=(char *)malloc(sizeof(char)*6 );
     buf2=(char *)malloc(sizeof(char)*6);	

     memset(buf ,0,16);
     memset(buf1,0,6 );  
     memset(buf2,0,6);
	
     i=1;
	
	
	strcpy(buf,"");	  //清空		
	strcpy(buf,"cmd");//帧头

     //添加命令前缀	
	switch(command_num)
	{
		//驱动电机动作
		case Stop:               strcat(buf,"s");break;//Stop all motors  				
		case GoAhead:	          strcat(buf,"f");break;//Go forward 
		case GoBack:	 		strcat(buf,"b");break;//Go Back 
		case TurnRight:		strcat(buf,"r");break;//TurnRight
		case TurnLeft: 		strcat(buf,"l");break;//TurnLeft	

		//捡球支架
		case TurnDownPickerBody:	strcat(buf,"d");break;//TurnDownPicker	
		case HandOnPickerBody:	strcat(buf,"u");break;//HandOnPicker			
		case TurnUpPickerBody:	strcat(buf,"u");break;//TurnUpPicker
          //捡球飞轮
          case TurnUpFlyWheel:     strcat(buf,"p");break;//TurnUpPicker
		case TurnDownFlyWheel:   strcat(buf,"p");break;//TurnUpPicker

		//风扇
		case TurnOnFan:	 	strcat(buf,"w");break;//Turn On Wind		
		case TurnOffFan:	 	strcat(buf,"w");break;//Turn Off Wind	\		
		
		default:
		break;
	}

     //将左轮速度值转换为4个字符-7199~~7199,(A-F不使用)*********************************
     //[0,SPEED_STOP-200) 设置为0
     if(speedLeft>=0 && speedLeft < SPEED_STOP)
          strcat(buf1,"0000");
     //[SPEED_STOP-200,1000)     
     else if(speedLeft>=SPEED_STOP && speedLeft<1000) 
     {
          strcat(buf1,"0"); 
          sprintf(speed_buf,"%3d",speedLeft);
          strcat(buf1,speed_buf);//速度SPEED_STOP - 1000        
     }
     //[1000,7199]      
     else if(speedLeft>=1000 && speedLeft<=7199)
     {
          sprintf(speed_buf,"%4d",speedLeft);
          strcat(buf1,speed_buf);//速度00-99         
     }
     //其他,可能错误的速度值
     else
     {
          strcat(buf1,"0000"); 
     }   
     //将右轮速度值转换为4个字符-7199~~7199,(A-F不使用)*********************************
     //[0,SPEED_STOP-200) 设置为0
     if(speedRight>=0 && speedRight < SPEED_STOP)
          strcat(buf2,"0000");
     //[SPEED_STOP-200,1000)     
     else if(speedRight>=SPEED_STOP && speedRight<1000) 
     {
          strcat(buf2,"0"); 
          sprintf(speed_buf,"%3d",speedRight);
          strcat(buf2,speed_buf);//速度SPEED_STOP - 1000        
     }
     //[1000,7199]      
     else if(speedRight>=1000 && speedRight<=7199)
     {
          sprintf(speed_buf,"%4d",speedRight);
          strcat(buf2,speed_buf);//速度00-99         
     }
     //其他,可能错误的速度值
     else
     {
          strcat(buf2,"0000"); 
     }

     //按照命令类型,编制不同的发送指令:
     //s,p,w,d,u的命令格式为 "cmd*1234#",f,b,l,r的命令格式为 "cmd*1234,1234#"
     switch(command_num)
     {
          case Stop:
               strcat(buf,"0000");
               break;

		case TurnDownPickerBody:	
		case HandOnPickerBody:			
		case TurnUpPickerBody:          
          case TurnUpFlyWheel: 
		case TurnOffFlyWheel: 
		case TurnOnFan:		
		case TurnOffFan:
          case TurnRight:		
		case TurnLeft: 
               strcat(buf,buf1);          
               strcat(buf,"#");//帧尾
               write(fd,buf,10);  
               break;             
 		case GoAhead:	           
		case GoBack:			

               strcat(buf,buf1); 
               strcat(buf,",");//
               strcat(buf,buf2);
               strcat(buf,"#");//帧尾
               write(fd,buf,14);
               break;  
     }

     //usleep(100);
	
     #if DEBUG_Command
     //usleep(1000);
     printf("%d,%d\n",speedLeft,speedRight);
     printf("%s\n",buf);
     #endif
     //usleep(100*1000);	

     free(buf);
     buf=NULL;

     free(buf1);
     buf1=NULL;
 
     free(buf2);
     buf2=NULL;

     //close(fd);	
	return 0;
}

