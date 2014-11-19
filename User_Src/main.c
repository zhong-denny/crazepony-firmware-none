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
��д�ߣ�С��  (Camel)
����E-mail��375836945@qq.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2014-01-28
���ܣ�
1.�ɻ�Ӳ����ʼ��
2.������ʼ��
3.��ʱ����
4.�ȴ��жϵ���
------------------------------------
*/

#include "config.h"        //�������е�����ͷ�ļ�

#include "control.h"
#include "moto.h"
#include "extern_variable.h"

#define MAX_MOTOR_SPEED 500
S_FLOAT_ANGLE  groud_angle;

/********************************************
              �ɿ����������
���ܣ�                                        
1.��ʼ������Ӳ��
2.��ʼ��ϵͳ����
3.����ʱ��4�ȴ������жϵ���
4.����ʱ��3���ڹ㲥ʵʱ��̬�Լ������Ϣ
********************************************/
int main(void)
{
  int MotorSpeed, i;
  long Motor[4]; 
  long iMotor[4];
  
  MotorSpeed = 10;
  

  SystemClock_HSE(9);           //ϵͳʱ�ӳ�ʼ����ʱ��Դ�ⲿ����HSE
  //SystemClock_HSI(9);         //ϵͳʱ�ӳ�ʼ����ʱ��Դ�ڲ�HSI
  UART1_init(SysClock,115200); 	//����1��ʼ��
  NVIC_INIT();	                //�жϳ�ʼ��
  STMFLASH_Unlock();            //�ڲ�flash����
  LedInit();		                //IO��ʼ�� 
  delay_init(SysClock);         //�δ���ʱ��ʼ�� 
  BT_PowerInit();               //������Դ��ʼ����ɣ�Ĭ�Ϲر� 
  MotorInit();	                //����ʼ��
  BatteryCheckInit();           //��ص�ѹ����ʼ��
  IIC_Init();                   //IIC��ʼ��
  MPU6050_DMP_Initialize();     //��ʼ��DMP����
  PID_INIT();                   //PID������ʼ��  
  //HMC5883L_SetUp();           //��ʼ��������HMC5883L
  ParameterRead();              //Flash������ȡ
  PID_INIT();                   //PID������ʼ�� 
  NRF24L01_INIT();              //NRF24L01��ʼ��
  SetRX_Mode();                 //������ģ��Ϊ����ģʽ
  /////////////////////////
//   NRF24L01_RXDATA[30]=0xA5;
//   NRF24L01_RXDATA[27]=0xA5;//��������,�����ã����������poweron
  /////////////////////////
  PowerOn();                    //�����ȴ�  
  //BT_ATcmdWrite();              //����д����
  //BT_off();                     //�����ر�
  //ParameterWrite();             //д�������ڲ�ģ��eeprom
  printf("Init int 3\r\n");
  TIM3_Init(SysClock,1000);	    //��ʱ��3��ʼ�������Դ������
  printf("Init int 4\r\n");
  TIM4_Init(SysClock,1000);	    //��ʱ��4��ʼ������ʱ�������������ݣ�����PID�������ʱ����ʱ��ʯΪ1us��PID��������Ϊ4ms��������̬����Ƶ�� Ϊ250Hz    
  printf("Init int done\r\n");


  //wait for 10 sec
  //Delay( 3 * GL_Second);
  printf("****DEMOFLY...\r\n");

  //���浱ǰλ�ã�������ɺ�����ֹͣ�Աȸ߶�
  groud_angle.Yaw = Q_ANGLE.Yaw;
  groud_angle.Pitch = Q_ANGLE.Pitch;
  groud_angle.Roll = Q_ANGLE.Roll;  

  //��������ת�٣��ж��Ƿ���ɣ�����ɵ�һ���߶ȣ�ֹͣ�仯��

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

