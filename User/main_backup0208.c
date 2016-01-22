/*****************************	     	DAMH2		    	*****************************************************										
										           | |                     | |
											      RFL_1(6-PC6)     		    RHL_1(3-PB6)
														   | |                     | |
														RFL_2(8-PC7)     		    RHL_2(1-PB7)    
											 ____________________________________________
											|                                            |
											|	     RFL_3(7-PC8)	        	RHL_3(2-PB8)   |
											|		         ____________________            |
											|		        |										 | 	         |
											|		        |										 | 	         |
											|		        |										 | 	         |
											|		        |____________________|	         |
											|                                            |
											|     LFL_3(9-PA2)      	    LHL_3(15-PB11) |
											|____________________________________________|	
														LFL_2(10-PA1)      		  LHL_2(16-PB10)
														   | |                    | |
											    	LFL_1(11-PA0)      	    LHL_1(14-PB3)
														   | |                    | |

******************************************************************************************************
RC_position_Control
                         min_phi                       max_phi
degree                      0                             120
PWM_CCR_pulse              2710                           7390

angle_step = (7330-2770)/120 = 38
******************************************************************************************************
RC_Speed_Control
 
  w (degree/second) = SysTick_interval * 1/time *1/k * 120/(7390-2710)
							                           (systick_subroutine)
******************************************************************************************************
USART 
       PA9: TX                 PA10:RX
******************************************************************************************************
*note for programing: Dont use Delay in Systick functions, can cause other systick functions stop working
			                Bring the use-delay function to main so systick functions can work properly like interrupt
******************************************************************************************************/

#include "stm32f4xx.h"
#include "PWM.h"
#include "UART_DMA.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define SysTick_interval 10000
//LFL
#define phi0_LFL_1 5050  
#define phi0_LFL_2 5050 
#define phi0_LFL_3 5164

//LHL
#define phi0_LHL_1 5050
#define phi0_LHL_2 4860 
#define phi0_LHL_3 5164

//RFL
#define phi0_RFL_1 4898
#define phi0_RFL_2 5278
#define phi0_RFL_3 5392

//RHL
#define phi0_RHL_1 5050
#define phi0_RHL_2 4936
#define phi0_RHL_3 5202


#define angle_step 38

/************************Global variables ----------------------------------------------------------*/
//UART_DMA section
	//Tx
	#define txsize 7
	uint8_t txbuffer[txsize];

	//RX
	#define rxsize 56 //min = 7 channel * 7 bytes = 49
	uint8_t rxbuffer[rxsize];
	char RX_tempbuffer;
	uint8_t rxindex=0,i;
	float   rx_data;
	int8_t	X1_Analog,
					X2_Analog,
					Y1_Analog,
					Y2_Analog,
					LT_Analog,
					RT_Analog;
	uint16_t button;
	
uint16_t SysTicker = 0;
uint32_t CCR_now;


uint16_t phi_LFL_1,k_LFL_1,
				 phi_LFL_2,k_LFL_2,
				 phi_LFL_3,k_LFL_3;
uint16_t phi_LHL_1,k_LHL_1,
				 phi_LHL_2,k_LHL_2,
				 phi_LHL_3,k_LHL_3;
uint16_t phi_RFL_1,k_RFL_1,
				 phi_RFL_2,k_RFL_2,
				 phi_RFL_3,k_RFL_3;
uint16_t phi_RHL_1,k_RHL_1,
				 phi_RHL_2,k_RHL_2,
				 phi_RHL_3,k_RHL_3;

//Status feedback
int8_t   fb_LFL_1,fb_LFL_2,fb_LFL_3,
				 fb_LHL_1,fb_LHL_2,fb_LHL_3,
				 fb_RFL_1,fb_RFL_2,fb_RFL_3,
				 fb_RHL_1,fb_RHL_2,fb_RHL_3;

/************************Subroutines_Prototype****************************/
void Delay(uint32_t nCount);// t = nCount (ms)
//void delay(uint16_t time); //delay func using systick
void Zero_state (void);
void X360_button (void);
	void action1 (void);
	void action2 (void);
	void action3 (void);
void X360_Y1_Analog (void);
	void walk (void);

//Set
void LFL_1_set (int8_t phi, uint8_t k);
void LFL_2_set (int8_t phi, uint8_t k);
void LFL_3_set (int8_t phi, uint8_t k);

void LHL_1_set (int8_t phi, uint8_t k);
void LHL_2_set (int8_t phi, uint8_t k);
void LHL_3_set (int8_t phi, uint8_t k);

void RFL_1_set (int8_t phi, uint8_t k);
void RFL_2_set (int8_t phi, uint8_t k);
void RFL_3_set (int8_t phi, uint8_t k);

void RHL_1_set (int8_t phi, uint8_t k);
void RHL_2_set (int8_t phi, uint8_t k);
void RHL_3_set (int8_t phi, uint8_t k);

//Systick prototype
void feedback (uint16_t time);
//Execute
void LFL_1_execute(uint16_t time);
void LFL_2_execute(uint16_t time);
void LFL_3_execute(uint16_t time);

void LHL_1_execute(uint16_t time);
void LHL_2_execute(uint16_t time);
void LHL_3_execute(uint16_t time);

void RFL_1_execute(uint16_t time);
void RFL_2_execute(uint16_t time);
void RFL_3_execute(uint16_t time);

void RHL_1_execute(uint16_t time);
void RHL_2_execute(uint16_t time);
void RHL_3_execute(uint16_t time);

/************************MAIN*********************************************/
int main(void)
{
	TIM_Config();
	PWM_Config();
	UART1_DMA_CONFIG(txbuffer,txsize,rxbuffer,rxsize,57600);
	USART_config ();
	Zero_state ();
	if (SysTick_Config(SystemCoreClock / SysTick_interval)) // systick_interrupt occur every 0.1 ms (1000=1ms)
  {/* Capture error */ while (1);}
	while (1)
	{
		//control signal
		X360_button ();
		X360_Y1_Analog();
	}
}

/************************Systick_interrupt_0.1ms******************************************/
void SysTick_Handler(void)
{
	SysTicker ++;
	//actuator
	LFL_1_execute(1);
	LFL_2_execute(1);
	LFL_3_execute(1);
	
	LHL_1_execute(1);
	LHL_2_execute(1);
	LHL_3_execute(1);
	
	RFL_1_execute(1);
	RFL_2_execute(1);
	RFL_3_execute(1);
	
	RHL_1_execute(1);
	RHL_2_execute(1);
	RHL_3_execute(1);
		
	//feedback
	feedback (10);
}

uint16_t GetTicker(void)
{return SysTicker;}

/************************Interrupts*****************************/
//UART RX 
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
			/* Read data from the receive data register */
			RX_tempbuffer = USART_ReceiveData(USART1);
			rxbuffer[rxindex] = RX_tempbuffer;
			rxindex++;
			if (rxindex == rxsize) rxindex=0;
			for (i=0;i<=rxsize-6;i++)
			{
				if ((rxbuffer[i] == 0xFF) && (rxbuffer[i + 6] == 0xFE))
				{
					memcpy(&rx_data,&rxbuffer[i+2],4);
					switch (rxbuffer[i+1])
					{
						case 0xE0:button = (uint16_t)rx_data;break;
						case 0xE1:X1_Analog = (int8_t)rx_data;break;
						case 0xE2:X2_Analog = (int8_t)rx_data;break;
						case 0xE3:Y1_Analog = (int8_t)rx_data;break;
						case 0xE4:Y2_Analog = (int8_t)rx_data;break;
						case 0xE5:LT_Analog = (int8_t)rx_data;break;
						case 0xE6:RT_Analog = (int8_t)rx_data;break;
					}
				}
			}
	}
}
/************************Subroutines****************************/
void Delay(uint32_t nCount)
{
  nCount = 42000 *nCount;
	while(nCount--);// tdelay (ms)= nCount [(4/168Mhz*1000)*nCount*42000]
}
//void delay(uint16_t time) //delay func using systick time 0.1ms
//{
//	uint16_t counter = GetTicker();
//	while ((uint16_t)(counter+time)!=GetTicker());// sticker go to 65355 then 0 then count up
//}
void Zero_state (void)
{
		LFL_3_set (0,2);
		LHL_3_set (0,2);
		RFL_3_set (0,2);
		RHL_3_set (0,2);

		LFL_2_set (0,2);
		LHL_2_set (0,2);
		RFL_2_set (0,2);
		RHL_2_set (0,2);
		
		LFL_1_set (0,2);
		LHL_1_set (0,2);
		RFL_1_set (0,2);
		RHL_1_set (0,2);
}
void X360_button (void)
{
		switch (button)
					{
						case 1:action1();break;
						case 2:action2();break;
					}
}
	void action1 (void)
	{	
			if (fb_LHL_1==0)
			{	
				LFL_2_set (-50,4);
				LFL_1_set (50,4);
				LHL_2_set (-50,4);
				LHL_1_set (50,4);
			}
			if (fb_LHL_1==50)
			{	
				LFL_2_set (0,4);
				LFL_1_set (0,4);
				LHL_2_set (0,4);
				LHL_1_set (0,4);
			}
	}

	void action2 (void)
	{
			if (fb_RHL_1 == 0)
			{
				RFL_2_set (-50,4);
				RFL_1_set (50,4);
				RHL_2_set (-50,4);
				RHL_1_set (50,4);
			}
			if (fb_RHL_1 == 50)
			{
				RFL_2_set (0,4);
				RFL_1_set (0,4);
				RHL_2_set (0,4);
				RHL_1_set (0,4);
			}
	}

	void action3 (void)
	{
		
	}
void X360_Y1_Analog (void)
{
	while (Y1_Analog > 20)
	{
		walk();
	}		
}
	void walk (void)
	{
			LFL_2_set (-25,4);
			LFL_1_set (40,4);
			Delay (3000);
			LFL_2_set (30,4);
			Delay (200);
			LFL_1_set (0,4);
			Delay (3000);
	}
// Systick subroutines
void feedback (uint16_t time)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		fb_LFL_1 = +((int16_t)TIM_GetCapture1(TIM5)-phi0_LFL_1)/angle_step;
		fb_LFL_2 = -((int16_t)TIM_GetCapture2(TIM5)-phi0_LFL_2)/angle_step;
		fb_LFL_3 = -((int16_t)TIM_GetCapture3(TIM5)-phi0_LFL_3)/angle_step;
		
		fb_LHL_1 = +((int16_t)TIM_GetCapture2(TIM2)-phi0_LHL_1)/angle_step;
		fb_LHL_2 = -((int16_t)TIM_GetCapture3(TIM2)-phi0_LHL_2)/angle_step;
		fb_LHL_3 = -((int16_t)TIM_GetCapture4(TIM2)-phi0_LHL_3)/angle_step;
		
		fb_RFL_1 = -((int16_t)TIM_GetCapture1(TIM3)-phi0_RFL_1)/angle_step;
		fb_RFL_2 = +((int16_t)TIM_GetCapture2(TIM3)-phi0_RFL_2)/angle_step;
		fb_RFL_3 = +((int16_t)TIM_GetCapture3(TIM3)-phi0_RFL_3)/angle_step;
		
		fb_RHL_1 = -((int16_t)TIM_GetCapture1(TIM4)-phi0_RHL_1)/angle_step;
		fb_RHL_2 = +((int16_t)TIM_GetCapture2(TIM4)-phi0_RHL_2)/angle_step;
		fb_RHL_3 = +((int16_t)TIM_GetCapture3(TIM4)-phi0_RHL_3)/angle_step;
	}
}
           /************************ Set ****************************/
void LFL_1_set (int8_t phi, uint8_t k) //phi = angle (degree)
{
	phi_LFL_1= phi0_LFL_1 +phi*angle_step;
	k_LFL_1=k;
}
void LFL_2_set (int8_t phi, uint8_t k) 
{
	phi_LFL_2= phi0_LFL_2 -phi*angle_step;
	k_LFL_2=k;
}
void LFL_3_set (int8_t phi, uint8_t k) 
{
	phi_LFL_3= phi0_LFL_3 -phi*angle_step;
	k_LFL_3=k;
}

void LHL_1_set (int8_t phi, uint8_t k) //phi = angle (degree)
{
	phi_LHL_1= phi0_LHL_1 +phi*angle_step;
	k_LHL_1=k;
}
void LHL_2_set (int8_t phi, uint8_t k) 
{
	phi_LHL_2= phi0_LHL_2 -phi*angle_step;
	k_LHL_2=k;
}
void LHL_3_set (int8_t phi, uint8_t k) 
{
	phi_LHL_3= phi0_LHL_3 -phi*angle_step;
	k_LHL_3=k;
}

void RFL_1_set (int8_t phi, uint8_t k) //phi = angle (degree)
{
	phi_RFL_1= phi0_RFL_1 -phi*angle_step;
	k_RFL_1=k;
}
void RFL_2_set (int8_t phi, uint8_t k) 
{
	phi_RFL_2= phi0_RFL_2 +phi*angle_step;
	k_RFL_2=k;
}
void RFL_3_set (int8_t phi, uint8_t k) 
{
	phi_RFL_3= phi0_RFL_3 +phi*angle_step;
	k_RFL_3=k;
}

void RHL_1_set (int8_t phi, uint8_t k) //phi = angle (degree)
{
	phi_RHL_1= phi0_RHL_1 -phi*angle_step;
	k_RHL_1=k;
}
void RHL_2_set (int8_t phi, uint8_t k) 
{
	phi_RHL_2= phi0_RHL_2 +phi*angle_step;
	k_RHL_2=k;
}
void RHL_3_set (int8_t phi, uint8_t k) 
{
	phi_RHL_3= phi0_RHL_3 +phi*angle_step;
	k_RHL_3=k;
}

					/************************ Execute - Systicks ****************************/
//LFL
void LFL_1_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture1(TIM5);
		if (phi_LFL_1>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LFL_1) ==0)
				TIM_SetCompare1(TIM5, CCR_now);
		}
		else if (phi_LFL_1<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LFL_1) ==0)
				TIM_SetCompare1(TIM5, CCR_now);
		}
	}
}
void LFL_2_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture2(TIM5);
		if (phi_LFL_2>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LFL_2) ==0)
				TIM_SetCompare2(TIM5, CCR_now);
		}
		else if (phi_LFL_2<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LFL_2) ==0)
				TIM_SetCompare2(TIM5, CCR_now);
		}
	}
}
void LFL_3_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture3(TIM5);
		if (phi_LFL_3>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LFL_3) ==0)
				TIM_SetCompare3(TIM5, CCR_now);
		}
		else if (phi_LFL_3<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LFL_3) ==0)
				TIM_SetCompare3(TIM5, CCR_now);
		}
	}
}

//LHL
void LHL_1_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture2(TIM2);
		if (phi_LHL_1>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LHL_1) ==0)
				TIM_SetCompare2(TIM2, CCR_now);
		}
		else if (phi_LHL_1<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LHL_1) ==0)
				TIM_SetCompare2(TIM2, CCR_now);
		}
	}
}
void LHL_2_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture3(TIM2);
		if (phi_LHL_2>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LHL_2) ==0)
				TIM_SetCompare3(TIM2, CCR_now);
		}
		else if (phi_LHL_2<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LHL_2) ==0)
				TIM_SetCompare3(TIM2, CCR_now);
		}
	}
}
void LHL_3_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture4(TIM2);
		if (phi_LHL_3>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LHL_3) ==0)
				TIM_SetCompare4(TIM2, CCR_now);
		}
		else if (phi_LHL_3<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LHL_3) ==0)
				TIM_SetCompare4(TIM2, CCR_now);
		}
	}
}

//RFL
void RFL_1_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture1(TIM3);
		if (phi_RFL_1>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RFL_1) ==0)
				TIM_SetCompare1(TIM3, CCR_now);
		}
		else if (phi_RFL_1<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RFL_1) ==0)
				TIM_SetCompare1(TIM3, CCR_now);
		}
	}
}
void RFL_2_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture2(TIM3);
		if (phi_RFL_2>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RFL_2) ==0)
				TIM_SetCompare2(TIM3, CCR_now);
		}
		else if (phi_RFL_2<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RFL_2) ==0)
				TIM_SetCompare2(TIM3, CCR_now);
		}
	}
}
void RFL_3_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture3(TIM3);
		if (phi_RFL_3>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RFL_3) ==0)
				TIM_SetCompare3(TIM3, CCR_now);
		}
		else if (phi_RFL_3<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RFL_3) ==0)
				TIM_SetCompare3(TIM3, CCR_now);
		}
	}
}

//RHL
void RHL_1_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture1(TIM4);
		if (phi_RHL_1>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RHL_1) ==0)
				TIM_SetCompare1(TIM4, CCR_now);
		}
		else if (phi_RHL_1<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RHL_1) ==0)
				TIM_SetCompare1(TIM4, CCR_now);
		}
	}
}
void RHL_2_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture2(TIM4);
		if (phi_RHL_2>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RHL_2) ==0)
				TIM_SetCompare2(TIM4, CCR_now);
		}
		else if (phi_RHL_2<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RHL_2) ==0)
				TIM_SetCompare2(TIM4, CCR_now);
		}
	}
}
void RHL_3_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture3(TIM4);
		if (phi_RHL_3>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RHL_3) ==0)
				TIM_SetCompare3(TIM4, CCR_now);
		}
		else if (phi_RHL_3<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RHL_3) ==0)
				TIM_SetCompare3(TIM4, CCR_now);
		}
	}
}
