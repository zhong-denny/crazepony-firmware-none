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
*/
/* Control.c file
��д�ߣ�С��  (Camel)
����E-mail��375836945@qq.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2014-01-28
���ܣ�
1.PID������ʼ��
2.���ƺ���

------------------------------------
*/
#include "control.h"
#include "moto.h"
#include "math.h"
#include "sys_fun.h"
#include "mpu6050.h"
#include "imu.h"
#include "extern_variable.h"
#include "led.h"
#include "stmflash.h"
#include "ReceiveData.h"
#include "DMP.h"
#include "Battery.h"
#include "stdio.h"
#include "BT.h"

//----PID�ṹ��ʵ����----
PID_Typedef pitch_angle_PID;	//�ǶȻ���PID
PID_Typedef pitch_rate_PID;		//�����ʻ���PID

PID_Typedef roll_angle_PID;
PID_Typedef roll_rate_PID;

PID_Typedef yaw_angle_PID;
PID_Typedef yaw_rate_PID;

float gyroxGloble = 0;
float gyroyGloble = 0;


S_FLOAT_XYZ DIF_ACC;		//ʵ��ȥ�������ļ��ٶ�
S_FLOAT_XYZ EXP_ANGLE;	//�����Ƕ�	
S_FLOAT_XYZ DIF_ANGLE;	//ʵ�����������ĽǶ�	

void ControlerDemo(void)
{     
    static char Counter_Cnt=0;
    Counter_Cnt++;
    DMP_Routing();	        //DMP �߳�  ���е����ݶ����������
    DMP_getYawPitchRoll();  //��ȡ ��̬��
    #ifndef Debug
    Send_AtitudeToPC();     
    #else
    #endif    
    if(Counter_Cnt==5)
    {
	    Counter_Cnt=0;
		//PID_Calculate();     //=2ʱ����һ��,Ƶ��200HZ	
    }
}

//��������Controler()
//���룺��
//���: ��
//�������ɻ����ƺ������壬����ʱ������
//���ߣ���
//��ע��û�����У����鲻��
void Controler(void)
{     
    static char Counter_Cnt=0;
    Counter_Cnt++;
    DMP_Routing();	        //DMP �߳�  ���е����ݶ����������
    DMP_getYawPitchRoll();  //��ȡ ��̬��
    /*******************����λ��������̬��Ϣ�����Ҫ��PC��λ����ʵʱ��̬,�꿪�ؿ���***************/
    #ifndef Debug
    Send_AtitudeToPC();     
    #else
    #endif 
  
    if(Counter_Cnt==5)
    {
	    Counter_Cnt=0;
		Nrf_Irq();           //��2.4G���տ���Ŀ�����
	    //ReceiveDataFormUART();//������͸��ģ����տ���Ŀ���������2.4G���տ���ֻ��ѡ��һ
	    PID_Calculate();     //=2ʱ����һ��,Ƶ��200HZ	
    }
}







//-----------λ��ʽPID-----------
void PID_Postion_Cal(PID_Typedef * PID,float target,float measure,int32_t dertT)
{
	//-----------λ��ʽPID-----------
	//���=����ֵ-����ֵ
	PID->Error=target-measure;
	
	PID->Integ+=(double)PID->Error*dertT/1000000.0;

	PID->Deriv=PID->Error-PID->PreError;
	
	PID->Output=PID->P*PID->Error+PID->I*PID->Integ+PID->D*PID->Deriv;
	
	PID->PreError=PID->Error;
	
}

//��������PID_Calculate()
//���룺��
//���: ��
//�������ɻ�������PIDʵ�ֺ���
//���ߣ���
//��ע��û�����У����鲻��
float YawLock = 0;
uint8_t YawLockState  = 0;
int YawLockRange = 10; //��������ƫ������������λ�����ķ�Χ
void PID_Calculate(void)
{  
    static float Thr=0,Rool=0,Pitch=0,Yaw=0;
    static int PIDcounter=0;
    long Motor[4];   //������PWM���飬�ֱ��ӦM1-M4
  /*********************************************************
     ����������̬��ʵ����̬�Ĳ�ֵ
    *********************************************************/
    EXP_ANGLE.X = (float)(RC_DATA.ROOL);
    EXP_ANGLE.Y = (float)(RC_DATA.PITCH);
    EXP_ANGLE.Z = (float)(RC_DATA.YAW);

    DIF_ANGLE.X = EXP_ANGLE.X - Q_ANGLE.Roll;
    DIF_ANGLE.X = DIF_ANGLE.X;
    
    DIF_ANGLE.Y = EXP_ANGLE.Y - Q_ANGLE.Pitch;
    DIF_ANGLE.Y = DIF_ANGLE.Y;

    DIF_ACC.Z =  DMP_DATA.dmp_accz - g;    
  
    //ƫ����λ�ж�
    if(EXP_ANGLE.Z <YawLockRange && EXP_ANGLE.Z>-YawLockRange)
    {
        if(YawLockState == 0x00)
        {
          YawLock = Q_ANGLE.Yaw;
          YawLockState = 0xff;
        }
    }
    else YawLockState = 0x00;
    
    
    
    PIDcounter++;
    if(PIDcounter == 2)
    {
            PIDcounter = 0;
          /*********************************************************
           PID�����㷨����
          *********************************************************/
          //------------��������------------
          //��������ԭ��Ϊ���ں��⣬���������ڻ�ʱ���⻷��PID����Ϊ0
          //�⻷�� �ơ�����Ϊ�Ƕ�,���Ϊ���ٶȡ�PID->Output��Ϊ�ڻ������롣
          PID_Postion_Cal(&pitch_angle_PID,EXP_ANGLE.Y,Q_ANGLE.Pitch,0);   
          //�⻷�� �ơ�����Ϊ�Ƕ�,���Ϊ���ٶȡ�PID->Output��Ϊ�ڻ������롣
          PID_Postion_Cal(&roll_angle_PID,EXP_ANGLE.X,Q_ANGLE.Roll,0);
    }
    
    
    if(YawLockState == 0x00) YawLock = Q_ANGLE.Yaw; //ƫ������״̬�ж�  
    else  EXP_ANGLE.Z = 0;
    //�⻷�� �ơ�����Ϊ�Ƕ�,���Ϊ���ٶȡ�PID->Output��Ϊ�ڻ������롣
    PID_Postion_Cal(&yaw_angle_PID,YawLock,Q_ANGLE.Yaw,0);
    
    
    
    //�ڻ����ƣ�����Ϊ���ٶȣ����ΪPWM����
    PID_Postion_Cal(&pitch_rate_PID,pitch_angle_PID.Output,gyroyGloble,0);
    //��������ԭ��Ϊ���ں��⣬���������ڻ�ʱ���⻷��PID����Ϊ0
    //�ڻ����ƣ�����Ϊ���ٶȣ����ΪPWM����
    PID_Postion_Cal(&roll_rate_PID,roll_angle_PID.Output,gyroxGloble,0);
    //��������ԭ��Ϊ���ں��⣬���������ڻ�ʱ���⻷��PID����Ϊ0
    //�ڻ����ƣ�����Ϊ���ٶȣ����ΪPWM����
    PID_Postion_Cal(&yaw_rate_PID,-yaw_angle_PID.Output - EXP_ANGLE.Z,DMP_DATA.GYROz,0);
    //��������ԭ��Ϊ���ں��⣬���������ڻ�ʱ���⻷��PID����Ϊ0
    
    
    
    //�������Ŷ���
    //Thr = 0.001*RC_DATA.THROTTLE*RC_DATA.THROTTLE;   //RC_DATA.THROTTLEΪ0��1000,��ҡ����������ת��Ϊ�°���������
    Thr = RC_DATA.THROTTLE;
    Thr -=  80*DIF_ACC.Z;                             
    
    Pitch = pitch_rate_PID.Output;
    Rool  = roll_rate_PID.Output;
    Yaw   = yaw_rate_PID.Output; 
    
   //�����ֵ�ںϵ��ĸ���� 
    Motor[2] = (int16_t)(Thr - Pitch -Rool- Yaw );    //M3  
    Motor[0] = (int16_t)(Thr + Pitch +Rool- Yaw );    //M1
    Motor[3] = (int16_t)(Thr - Pitch +Rool+ Yaw );    //M4 
    Motor[1] = (int16_t)(Thr + Pitch -Rool+ Yaw );    //M2    
    
    if((FLY_ENABLE==0xA5))MotorPwmFlash(Motor[0],Motor[1],Motor[2],Motor[3]);   
    else                  MotorPwmFlash(0,0,0,0);//����ɻ��������ʱͻȻ��ת 
    if(NRF24L01_RXDATA[10]==0xA5) MotorPwmFlash(5,5,Motor[2],Motor[3]); //һ�����������������ȣ����Թ��ܣ���Ҫ��
    
    
    
    
     
}


#define PIDParameterAdd   0    //PID����д���׵�ַΪ
#define BTParameterAdd    32   //��������д��Flash��ַΪ

Parameter_Typedef PIDParameter;//ʵ����һ��PID��Flash����
Parameter_Typedef BTParameter; //ʵ����һ������Flash����

//��������ParameterWrite()
//���룺��
//��������յ���ַ29���ֽ�Ϊ0xA5ʱ������1�����򷵻�0
//�������ɻ������󣬵���⵽д�����ģʽʱ��д������
//���ߣ���
//��ע��û�����У����鲻��
char  ParameterWrite()
{
//         PIDParameter.WriteBuf[0] = 23;
//         PIDParameter.WriteBuf[1] = 1;
//         PIDParameter.WriteBuf[2] = 4;
//   
//         BTParameter.WriteBuf[1]  = 0;
//         BTParameter.WriteBuf[2]  = 0;
  
        //STMFLASH_Write(STM32_FLASH_BASE+STM32_FLASH_OFFEST+PIDParameterAdd,PIDParameter.WriteBuf,3); //PID ����д��Flash
        STMFLASH_Write(STM32_FLASH_BASE+STM32_FLASH_OFFEST+BTParameterAdd,BTParameter.WriteBuf,3);  //�������ò���д��Flash
       
return 0;
}

//��������ParameterRead()
//���룺��
//�������
//��������ʼ��ʱ����ȡ��λ�����һ���趨�Ĳ���
//���ߣ���
//��ע��û�����У����鲻��
void  ParameterRead()
{      
  //STMFLASH_Read(STM32_FLASH_BASE+STM32_FLASH_OFFEST+PIDParameterAdd,PIDParameter.ReadBuf,3);
  STMFLASH_Read(STM32_FLASH_BASE+STM32_FLASH_OFFEST+BTParameterAdd,BTParameter.ReadBuf,3);
  
  printf("��FLASH�ж�ȡ����...\r\n");

}


/*********************************
��������PID�ͳ�ʼ��Ư��������Դ
*********************************/

//#define ParameterReadFromFlash

//��������PID_INIT()
//���룺��
//���: ��
//������PID������ʼ��
//���ߣ���
//��ע��û�����У����鲻��
void PID_INIT(void) 
{
     pitch_angle_PID.P = 3.5;
     pitch_angle_PID.I = 0;
     pitch_angle_PID.D = 0;

     pitch_rate_PID.P  = 0.5; 
     pitch_rate_PID.I  = 0; 
     pitch_rate_PID.D  = 1.5; 
////////////////////////////////////////////
     roll_angle_PID.P = 3.5;
     roll_angle_PID.I = 0;
     roll_angle_PID.D = 0;

     roll_rate_PID.P  = 0.5;
     roll_rate_PID.I  = 0; 
     roll_rate_PID.D  = 1.5; 
///////////////////////////////////////////
     yaw_angle_PID.P = 1;
     yaw_angle_PID.I = 0;
     yaw_angle_PID.D = 0;
  
     yaw_rate_PID.P  = 15;
     yaw_rate_PID.I  = 0; 
     yaw_rate_PID.D  = 0; 

     printf("PID��ʼ�����...\r\n");
  
}





