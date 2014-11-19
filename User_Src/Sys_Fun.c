#include "config.h"
#include "extern_variable.h"



char SysClock;       //����洢ϵͳʱ�ӱ�������λMHz

/********************************************
           ϵͳ�ж����ȼ�����
���ܣ�
1.�����ж����ȼ����ú���ͳһ��װΪ���ж����ȼ����ó�ʼ��
********************************************/
void NVIC_INIT(void)
{
    TimerNVIC_Configuration();//��ʱ���ж�����
    UART1NVIC_Configuration();//����1�ж�����
}
//����������ִ���������踴λ!�����������𴮿ڲ�����.		    
//������ʱ�ӼĴ�����λ		  
void MYRCC_DeInit(void)
{	
 	RCC->APB1RSTR = 0x00000000;//��λ����			 
	RCC->APB2RSTR = 0x00000000; 

  RCC->AHBENR = 0x00000014;  //˯��ģʽ�����SRAMʱ��ʹ��.�����ر�.	  
  RCC->APB2ENR = 0x00000000; //����ʱ�ӹر�.			   
  RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //ʹ���ڲ�����ʱ��HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //��λSW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //��λHSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //��λHSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //��λPLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //�ر������ж�		 
}


/********************************************
           ʹ���ڲ�DCO����ϵͳʱ��
���ܣ�
1.ʹ���ڲ�HSIʱ�Ӷ���Ƶ��4MHz����ΪPLL����
2.PLL��Ƶϵ��PLLMUL<=9(ʵ�ʵ���13ʱ������������Ƶ�ڲ�ʱ��)
3.���������PLLMUL��PLL��Ƶϵ��
4.��ע���ٷ��ֲ���˵��ʹ��HSIʱ����ߵ��ﵽ36M��ʵ�ʿ��Դﵽ52M��
********************************************/
char SystemClock_HSI(u8 PLL)
{
    RCC->CR|=1<<0;              //�ڲ�����ʱ��ʹ��
    RCC->CR|=0<<16;              //�ⲿ����ʱ�ӹر�
    RCC->CR|=1<<18;
    while(!((RCC->CR)&(1<<1))); //�ȴ��ڲ�ʱ���ȶ�����
    RCC->CFGR|=(PLL-2)<<18;     //PPL��Ƶϵ��
    RCC->CFGR|=0<<16;           //PPL����ʱ��Դ,HSI����Ƶ����ΪPLL����Դ=4MHz
    RCC->CR|=1<<24;             //PLLʹ��
    while(!((RCC->CR)&(1<<25)));//�ȴ�PLL�ȶ�
    RCC->CFGR|=2<<0;            //ϵͳʱ��Դ���ã�PLL�����Ϊϵͳʱ��
    SysClock=4*PLL;             //����ϵͳʱ�ӣ���λMHz
    return SysClock;
}


/********************************************
           ʹ���ⲿ������Ϊϵͳʱ��Դ
���ܣ�
1.ʹ���ⲿHSEʱ��8M��ΪPLL����
2.PLL��Ƶϵ��PLLMUL<=9(ʵ�ʵ���16ʱ������������Ƶ�ⲿʱ��)
3.���������PLLMUL��PLL��Ƶϵ��
4.��ע���ٷ��ֲ���˵��ʹ��HSE��Ϊϵͳʱ��Դʱ����߿ɱ�Ƶ��72MHz������ʵ�ʿ��Ա�Ƶ��128Mϵͳ�����ȶ�
********************************************/
//ϵͳʱ�ӳ�ʼ������
//pll:ѡ��ı�Ƶ������2��ʼ�����ֵΪ16		
//ʱ��ԴΪ�ⲿ����
//��ע������������8M����ʱ����ֻ��ʹ���ⲿ8M������Ϊʱ��Դ��
//      ���ڲ���HSI����ʹ���ҷ���û������������λ��ɶ�취û
char SystemClock_HSE(u8 PLL)
{
	unsigned char temp=0;   
	MYRCC_DeInit();		    //��λ������������
 	RCC->CR|=1<<16;       //�ⲿ����ʱ��ʹ��HSEON
	while(!(RCC->CR>>17));//�ȴ��ⲿʱ�Ӿ���
	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;//����2����λ
	RCC->CFGR|=PLL<<18;   //����PLLֵ 2~16
	RCC->CFGR|=1<<16;	    //PLLSRC ON 
	FLASH->ACR|=0x32;	    //FLASH 2����ʱ����
	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//�ȴ�PLL����
	RCC->CFGR|=0x00000002;//PLL��Ϊϵͳʱ��	 
	while(temp!=0x02)     //�ȴ�PLL��Ϊϵͳʱ�����óɹ�
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}   

  SysClock=(PLL+2)*8;
  return SysClock;
}	

/********************************************
              ����LED�ĸ���״̬
********************************************/
void PowerOn()
{
  char i;            //ѭ������
  while(NRF24L01_RXDATA[30]!=0xA5)//��֤�յ�һ�����������ݰ�32���ֽ�,�ټ�������ĳ���
    {
        Nrf_Irq();printf("�ȴ�ң�ؽ���...\r\n");
        LedA_on;LedB_on;LedC_on;LedD_on;
		Delay(1*GL_Second);
		LedA_off;LedB_off;LedC_off;LedD_off;
		Delay(5*GL_Second);
    }
  printf("�Ѽ�⵽ң���ź�...\r\n");

    for(i=0;i<4;i++)//ѭ����˸4��
    {
    LedA_on;LedB_off;LedC_off;LedD_off;
    Delay(900000);
    LedA_off;LedB_on;LedC_off;LedD_off;
    Delay(900000);
    LedA_off;LedB_off;LedC_on;LedD_off;
    Delay(900000);
    LedA_off;LedB_off;LedC_off;LedD_on;
    Delay(900000);
    }
    while(NRF24L01_RXDATA[27]!=0xA5)//��֤�յ�һ�����������ݰ�32���ֽ�,�ټ�������ĳ���,�ȴ�����
    {
        Nrf_Irq();printf("�ȴ�����...\r\n");
        LedA_on;LedB_on;LedC_on;LedD_on;Delay(100);LedA_off;LedB_off;LedC_off;LedD_off;Delay(9000);
    }
    for(i=0;i<3;i++)//�����ɹ���������˸3����ʾ
    {
    LedA_on;LedB_on;LedC_on;LedD_on;
    Delay(900000);
    LedA_off;LedB_off;LedC_off;LedD_off;
    Delay(900000);
    }
    printf("�����ɹ�,�������ģʽ...\r\n");
}











