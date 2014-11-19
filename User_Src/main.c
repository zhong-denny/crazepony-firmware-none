/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
main.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.飞机硬件初始化
2.参数初始化
3.定时器开
4.等待中断到来
------------------------------------
*/

#include "config.h"        //包含所有的驱动头文件

#include "control.h"
#include "moto.h"
#include "extern_variable.h"

#define MAX_MOTOR_SPEED 500
S_FLOAT_ANGLE  groud_angle;

/********************************************
              飞控主函数入口
功能：                                        
1.初始化各个硬件
2.初始化系统参数
3.开定时器4等待数据中断到来
4.开定时器3串口广播实时姿态以及相关信息
********************************************/
int main(void)
{
  int MotorSpeed, i;
  long Motor[4]; 
  long iMotor[4];
  
  MotorSpeed = 10;
  

  SystemClock_HSE(9);           //系统时钟初始化，时钟源外部晶振HSE
  //SystemClock_HSI(9);         //系统时钟初始化，时钟源内部HSI
  UART1_init(SysClock,115200); 	//串口1初始化
  NVIC_INIT();	                //中断初始化
  STMFLASH_Unlock();            //内部flash解锁
  LedInit();		                //IO初始化 
  delay_init(SysClock);         //滴答延时初始化 
  BT_PowerInit();               //蓝牙电源初始化完成，默认关闭 
  MotorInit();	                //马达初始化
  BatteryCheckInit();           //电池电压监测初始化
  IIC_Init();                   //IIC初始化
  MPU6050_DMP_Initialize();     //初始化DMP引擎
  PID_INIT();                   //PID参数初始化  
  //HMC5883L_SetUp();           //初始化磁力计HMC5883L
  ParameterRead();              //Flash参数读取
  PID_INIT();                   //PID参数初始化 
  NRF24L01_INIT();              //NRF24L01初始化
  SetRX_Mode();                 //设无线模块为接收模式
  /////////////////////////
//   NRF24L01_RXDATA[30]=0xA5;
//   NRF24L01_RXDATA[27]=0xA5;//跳过解锁,调试用，跳过下面的poweron
  /////////////////////////
  PowerOn();                    //开机等待  
  //BT_ATcmdWrite();              //蓝牙写配置
  //BT_off();                     //蓝牙关闭
  //ParameterWrite();             //写参数到内部模拟eeprom
  printf("Init int 3\r\n");
  TIM3_Init(SysClock,1000);	    //定时器3初始化，调试串口输出
  printf("Init int 4\r\n");
  TIM4_Init(SysClock,1000);	    //定时器4初始化，定时采样传感器数据，更新PID输出，定时器定时基石为1us，PID更新周期为4ms，所以姿态更新频率 为250Hz    
  printf("Init int done\r\n");


  //wait for 10 sec
  //Delay( 3 * GL_Second);
  printf("****DEMOFLY...\r\n");

  //保存当前位置，用于起飞后悬空停止对比高度
  groud_angle.Yaw = Q_ANGLE.Yaw;
  groud_angle.Pitch = Q_ANGLE.Pitch;
  groud_angle.Roll = Q_ANGLE.Roll;  

  //逐渐提高马达转速，判断是否起飞，当起飞到一定高度，停止变化。

  for (i=0;i<4;i++){
  	iMotor[i] = 1;
  }

  while (1)
  {
	if (MotorSpeed < MAX_MOTOR_SPEED){
		MotorSpeed ++;
	}
	for (i=0;i<4;i++){
		Motor[i] = (int16_t) (MotorSpeed * iMotor[i]); 
	}

    MotorPwmFlash(Motor[0],Motor[1],Motor[2],Motor[3]);   
    printf("Speed is %d\r\n", MotorSpeed);
    //DMP_DATA.dmp_accx;
    //DMP_DATA.dmp_accy;
    //DMP_DATA.dmp_accz;
   	Delay(  9000 );
  }

}

