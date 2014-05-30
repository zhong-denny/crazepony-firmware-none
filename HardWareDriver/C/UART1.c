
 /*    
  *      ____                      _____                  +---+
  *     / ___\                     / __ \                 | R |
  *    / /                        / /_/ /                 +---+
  *   / /   ________  ____  ___  / ____/___  ____  __   __
  *  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
  * / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
  * \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
  *                                                 / /
  *                                            ____/ /
  *                                           /_____/
  *                                       
  *  Crazyfile control firmware                                        
  *  Copyright (C) 2011-2014 Crazepony-II                                        
  *
  *  This program is free software: you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation, in version 3.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  *  GNU General Public License for more details.
  * 
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  *
  *
  * debug.c - Debugging utility functions
  *
  */
#include "UART1.h"
#include "stdio.h"


//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40

// USART Receiver buffer
#define RX_BUFFER_SIZE 100

u8 U1TxBuffer[258];
u8 U1TxCounter=0;
u8 U1count=0; 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 




/**************************ʵ�ֺ���********************************************
*����ԭ��:		void U1NVIC_Configuration(void)
*��������:		����1�ж�����
�����������
���������û��	
*******************************************************************************/
void U1NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure; 
        /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}



/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_UART1(u32 baudrate)
*��������:		��ʼ��UART1
���������u32 baudrate   ����RS232���ڵĲ�����
���������û��	
*******************************************************************************/
void Uart1_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRH&=0XFFFFF00F;//IO״̬����
	GPIOA->CRH|=0X000008B0;//IO״̬����
		  
	RCC->APB2RSTR|=1<<14;   //��λ����1
	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
	//����������
 	USART1->BRR=mantissa; // ����������	 
	USART1->CR1|=0X200C;  //1λֹͣ,��У��λ.
  U1NVIC_Configuration();//�ж�����
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	U1TxBuffer[U1count++] = DataToSend;  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 UART1_Get_Char(void)
*��������:		RS232����һ���ֽ�  һֱ�ȴ���ֱ��UART1���յ�һ���ֽڵ����ݡ�
���������		 û��
���������       UART1���յ�������	
*******************************************************************************/
u8 UART1_Get_Char(void)
{
	while (!(USART1->SR & USART_FLAG_RXNE));
	return(USART_ReceiveData(USART1));
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putc_Hex(uint8_t b)
*��������:		RS232��ʮ������ASCII��ķ�ʽ����һ���ֽ�����
				�Ƚ�Ŀ���ֽ����ݸ�4λת��ASCCII �����ͣ��ٽ���4λת��ASCII����
				��:0xF2 ������ " F2 "
���������
		uint8_t b   Ҫ���͵��ֽ�
���������û��	
*******************************************************************************/
void UART1_Putc_Hex(uint8_t b)
{
      /* �ж�Ŀ���ֽڵĸ�4λ�Ƿ�С��10 */
    if((b >> 4) < 0x0a)
        UART1_Put_Char((b >> 4) + '0'); //С��10  ,����Ӧ����0-9��ASCII
    else
        UART1_Put_Char((b >> 4) - 0x0a + 'A'); //���ڵ���10 ����Ӧ���� A-F

    /* �ж�Ŀ���ֽڵĵ�4λ �Ƿ�С��10*/
    if((b & 0x0f) < 0x0a)
        UART1_Put_Char((b & 0x0f) + '0');//С��10  ,����Ӧ����0-9��ASCII
    else
        UART1_Put_Char((b & 0x0f) - 0x0a + 'A');//���ڵ���10 ����Ӧ���� A-F
   UART1_Put_Char(' '); //����һ���ո�,�����ֿ������ֽ�
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putw_Hex(uint16_t w)
*��������:		RS232��ʮ������ASCII��ķ�ʽ����һ���ֵ�����.���Ƿ���һ��int
				��:0x3456 ������ " 3456 "
���������
		uint16_t w   Ҫ���͵���
���������û��	
*******************************************************************************/
void UART1_Putw_Hex(uint16_t w)
{
	//���͸�8λ����,����һ���ֽڷ���
    UART1_Putc_Hex((uint8_t) (w >> 8));
	//���͵�8λ����,����һ���ֽڷ���
    UART1_Putc_Hex((uint8_t) (w & 0xff));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putdw_Hex(uint32_t dw)
*��������:		RS232��ʮ������ASCII��ķ�ʽ����32λ������.
				��:0xF0123456 ������ " F0123456 "
���������
		uint32_t dw   Ҫ���͵�32λ����ֵ
���������û��	
*******************************************************************************/
void UART1_Putdw_Hex(uint32_t dw)
{
    UART1_Putw_Hex((uint16_t) (dw >> 16));
    UART1_Putw_Hex((uint16_t) (dw & 0xffff));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putw_Dec(uint16_t w)
*��������:		RS232��ʮ����ASCII��ķ�ʽ����16λ������.
				��:0x123 ����������ʮ�������� " 291 "
���������
		uint16_t w   Ҫ���͵�16λ����ֵ
���������û��	
*******************************************************************************/
void UART1_Putw_Dec(uint32_t w)
{
    uint32_t num = 100000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
            UART1_Put_Char('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}


volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile unsigned char rx_wr_index;
volatile unsigned char RC_Flag;
//------------------------------------------------------
void USART1_IRQHandler(void)
{
  unsigned char data;
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART1, U1TxBuffer[U1TxCounter++]);                    

    /* Clear the USART1 transmit interrupt */
    USART_ClearITPendingBit(USART1, USART_IT_TXE); 

    if(U1TxCounter == U1count)
    {
      /* Disable the USART1 Transmit interrupt */
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }    
  }

  else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
  data=USART_ReceiveData(USART1);
  if(data==0xa5)
  { 
	RC_Flag|=b_uart_head;
  rx_buffer[rx_wr_index++]=data;
  }
  else if(data==0x5a)
       { if(RC_Flag&b_uart_head)
	     { rx_wr_index=0;
		   RC_Flag&=~b_rx_over;
         }
         else
		  rx_buffer[rx_wr_index++]=data;
         RC_Flag&=~b_uart_head;
       }
	   else
	   { rx_buffer[rx_wr_index++]=data;
		 RC_Flag&=~b_uart_head;
		 if(rx_wr_index==rx_buffer[0])
	     {  
			RC_Flag|=b_rx_over;
          }
	   }
  if(rx_wr_index==RX_BUFFER_SIZE)
  rx_wr_index--;
  /* Clear the USART1 transmit interrupt */
  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}
