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
Tim.c file
��д�ߣ�С��  (Camel)
����E-mail��375836945@qq.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2014-01-28
���ܣ�
1.��ʼ����ʱ��3�Ͷ�ʱ��4
2.��ʱ��3-->���ڴ�ӡ���ֲ���
3.��ʱ��4-->��̬�����Լ�PID��������ڹؼ��жϣ�����ʱ��4�������ȼ��Լ������ȼ���Ϊ��ߺ��б�Ҫ
------------------------------------
*/
#include "tim.h"
#include "config.h"



int LedCounter;//LED��˸����ֵ
float Compass_HMC[3];

//�������
void TIM4_IRQHandler(void)		//1ms�ж�һ��,���ڳ����ȡ6050��
{
	/*
    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
    {     
          Controler(); //���ƺ���
          //HMC58X3_mgetValues(&Compass_HMC[0]);       
          LedCounter++;//led��˸����ֵ
          if(Battery.BatteryAD > Battery.BatteryADmin)//����ص�ѹ���趨ֵ֮��ʱ������ģʽ
          {
              if(LedCounter==10){ LedA_off;LedB_off;}   //ң�ض�ʹ�ܺ�������ʾ        
              else if(LedCounter==30){LedCounter=0;LedA_on;LedB_on;}
          }
          else //��ص�ѹ��ʱ��������ʾ
          {
              if(LedCounter==10){ LedA_off;LedB_off;LedC_off;LedD_off;}   //ң�ض�ʹ�ܺ�������ʾ        
              else if(LedCounter==20){LedCounter=0;LedA_on;LedB_on;LedC_on;LedD_on;}
          }
          if(LedCounter>=31)LedCounter=0;
          TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);   //����жϱ�־   
    }
    */

    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
    {     
          ControlerDemo(); //���ƺ���
          //HMC58X3_mgetValues(&Compass_HMC[0]);       
          LedCounter++;//led��˸����ֵ
          if(Battery.BatteryAD > Battery.BatteryADmin)//����ص�ѹ���趨ֵ֮��ʱ������ģʽ
          {
              if(LedCounter==10){ LedA_off;LedB_off;}   //ң�ض�ʹ�ܺ�������ʾ        
              else if(LedCounter==30){LedCounter=0;LedA_on;LedB_on;}
          }
          else //��ص�ѹ��ʱ��������ʾ
          {
              if(LedCounter==10){ LedA_off;LedB_off;LedC_off;LedD_off;}   //ң�ض�ʹ�ܺ�������ʾ        
              else if(LedCounter==20){LedCounter=0;LedA_on;LedB_on;LedC_on;LedD_on;}
          }
          if(LedCounter>=31)LedCounter=0;
          TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);   //����жϱ�־   
    }
}



int DebugCounter;             //��ӡ��Ϣ���ʱ��������ֵ


void TIM3_IRQHandler(void)		//��ӡ�жϷ������
{
    if( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
    {     
       

           
           Battery.BatteryAD  = GetBatteryAD();            //��ص�ѹ���  
           Battery.BatteryVal = Battery.Bat_K * (Battery.BatteryAD/4096.0) * Battery.ADRef;//ʵ�ʵ�ѹ ֵ����
#ifdef Debug
      DebugCounter++;
      if( DebugCounter==500)
            {
            DebugCounter=0;
            printf(" ******************************************************************\r\n");
            //printf(" *       ____                      _____                  +---+   *\r\n");
            //printf(" *      / ___\\                     / __ \\                 | R |   *\r\n");
            //printf(" *     / /                        / /_/ /                 +---+   *\r\n");
            //printf(" *    / /   ________  ____  ___  / ____/___  ____  __   __        *\r\n");
            //printf(" *   / /  / ___/ __ `/_  / / _ \\/ /   / __ \\/ _  \\/ /  / /        *\r\n");
            //printf(" *  / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /         *\r\n");
            //printf(" *  \\___/_/   \\__,_/ /___/\\___/_/    \\___ /_/ /_/____  /          *\r\n");
            //printf(" *                                                  / /           *\r\n");
            //printf(" *                                             ____/ /            *\r\n");
            //printf(" *                                            /_____/             *\r\n");
            //printf(" ******************************************************************\r\n");
            //printf("\r\n");
            printf(" Crazepony-II���棺ϵͳ��������...\r\n"); 
            printf("\r\n");
            printf("\r\n--->����ʵʱ��̬�㲥��Ϣ<---\r\n");
            printf("\r\n");
            printf(" ƫ����---> %5.2f��\r\n",(float)Q_ANGLE.Yaw);
            printf(" ������---> %5.2f��\r\n",(float)Q_ANGLE.Pitch);
            printf(" �����---> %5.2f��\r\n",(float) Q_ANGLE.Roll);
            printf(" ==================\r\n");
            printf(" X�������Ƕ�---> %5.2f��\r\n",(float)EXP_ANGLE.X);
            printf(" Y�������Ƕ�---> %5.2f��\r\n",(float)EXP_ANGLE.Y);
            printf(" Z�������Ƕ�---> %5.2f��\r\n",(float)EXP_ANGLE.Z);
            
            printf(" ==================\r\n");
            printf(" Y�����Ƕ�---> %5.2f��\r\n",(float)DIF_ANGLE.Y);
            printf(" X�����Ƕ�---> %5.2f��\r\n",(float)DIF_ANGLE.X);
            printf("==================\r\n");
            printf(" X����ٶ�---> %5.2fm/s2\r\n",(float) DMP_DATA.dmp_accx);
            printf(" Y����ٶ�---> %5.2fm/s2\r\n",(float) DMP_DATA.dmp_accy);
            printf(" Z����ٶ�---> %5.2fm/s2\r\n",(float) DMP_DATA.dmp_accz);
            
            printf(" ==================\r\n");
            printf(" X����ٶ�---> %5.2f ��/s\r\n",(float) DMP_DATA.dmp_gyrox);
            printf(" Y����ٶ�---> %5.2f ��/s\r\n",(float) DMP_DATA.dmp_gyroy);
            printf(" Z����ٶ�---> %5.2f ��/s\r\n",(float) DMP_DATA.dmp_gyroz);
            printf("==================\r\n");
            printf(" ���M1 PWMֵ---> %d\r\n",TIM2->CCR1);
            printf(" ���M2 PWMֵ---> %d\r\n",TIM2->CCR2);
            printf(" ���M3 PWMֵ---> %d\r\n",TIM2->CCR3);
            printf(" ���M4 PWMֵ---> %d\r\n",TIM2->CCR4);
            printf("==================\r\n");
            printf(" ��ص�ѹ---> %3.2fv\r\n",Battery.BatteryVal);//���ݲɼ�����ADֵ������ʵ�ʵ�ѹ��Ӳ�����ǶԵ�ؽ��з�ѹ���AD�ɼ��ģ����Խ��Ҫ����2
            printf("==================\r\n");
          
//             printf(" ---> %d\r\n",(int) PIDParameter.ReadBuf[0]);
//             printf(" ---> %d\r\n",(int) PIDParameter.ReadBuf[1]);
//             printf(" ---> %d\r\n",(int) PIDParameter.ReadBuf[2]);
//             
//             printf(" ---> %d\r\n",(int) BTParameter.ReadBuf[0]);
//             printf(" ---> %d\r\n",(int) BTParameter.ReadBuf[1]);
//             printf(" ---> %d\r\n",(int) BTParameter.ReadBuf[2]);
//             
            
// 
//             printf(" X�ų�ǿ��---> %5.2f ��/s\r\n",(float) Compass_HMC[0]);
//             printf(" Y�ų�ǿ��---> %5.2f ��/s\r\n",(float) Compass_HMC[1]);
//             printf(" Z�ų�ǿ��---> %5.2f ��/s\r\n",(float) Compass_HMC[2]);
//       


        }
#else      
             
#endif
        
        TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //����жϱ�־   
    }
}



//��ʱ��4��ʼ���������жϴ���PID
void TIM4_Init(char clock,int Preiod)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  //��ʱ��
    
    TIM_DeInit(TIM4);

    TIM_TimeBaseStructure.TIM_Period = Preiod;
    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//��ʱ1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM4,TIM_FLAG_Update);

    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM4,ENABLE);
    printf("��ʱ��4��ʼ�����...\r\n");
    
}	


//��ʱ��3��ʼ��
void TIM3_Init(char clock,int Preiod)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  //��ʱ��
    
    TIM_DeInit(TIM3);

    TIM_TimeBaseStructure.TIM_Period = Preiod;
    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//��ʱ1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM3,TIM_FLAG_Update);

    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM3,ENABLE);
  
    printf("��ʱ��3��ʼ�����...\r\n");
}		


void TimerNVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* NVIC_PriorityGroup 2 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    //TIM3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//��ʱ��3��Ϊ���ڴ�ӡ��ʱ�������ȼ�������̬����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ʱ��4��Ϊ��̬���㣬���ȼ����ڴ��ڴ�ӡ
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

} 

