/*****************************	     	DAMH2		    	*****************************************************										
										           | |                     | |
											      RFL_2(6-PC6)     		    RHL_2(3-PB6)
														   | |                     | |
														RFL_1(8-PC7)     		    RHL_1(1-PB7)    
											 ____________________________________________
											|                                            |
											|	     RFL_0(7-PC8)	        	RHL_0(2-PB8)   |
											|		         ____________________            |
											|		        |										 | 	         |
											|		        |										 | 	         |
											|		        |										 | 	         |
											|		        |____________________|	         |
											|                                            |
											|     LFL_0(9-PA2)      	    LHL_0(15-PB11) |
											|____________________________________________|	
														LFL_1(10-PA1)      		  LHL_1(16-PB10)
														   | |                    | |
											    	LFL_2(11-PA0)      	    LHL_2(14-PB3)
														   | |                    | |

******************************************************************************************************
RC_position_Control
                         min_phi                       max_phi
degree                      0                             120
PWM_CCR_pulse              2710                           7390

angle_step = (7390-2710)/120 = 39
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
#define phi0_LFL_2 2944
#define phi0_LFL_1 3100 
#define phi0_LFL_0 3100

//LHL
#define phi0_LHL_2 2788
#define phi0_LHL_1 3100 
#define phi0_LHL_0 7234

//RFL
#define phi0_RFL_2 7000
#define phi0_RFL_1 7000
#define phi0_RFL_0 7000

//RHL
#define phi0_RHL_2 7000
#define phi0_RHL_1 7000
#define phi0_RHL_0 3100


#define angle_step 39

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


uint16_t phi_LFL_2,k_LFL_2,
				 phi_LFL_1,k_LFL_1,
				 phi_LFL_0,k_LFL_0;
uint16_t phi_LHL_2,k_LHL_2,
				 phi_LHL_1,k_LHL_1,
				 phi_LHL_0,k_LHL_0;
uint16_t phi_RFL_2,k_RFL_2,
				 phi_RFL_1,k_RFL_1,
				 phi_RFL_0,k_RFL_0;
uint16_t phi_RHL_2,k_RHL_2,
				 phi_RHL_1,k_RHL_1,
				 phi_RHL_0,k_RHL_0;

//Status feedback
int8_t   fb_LFL_2,fb_LFL_1,fb_LFL_0,
				 fb_LHL_2,fb_LHL_1,fb_LHL_0,
				 fb_RFL_2,fb_RFL_1,fb_RFL_0,
				 fb_RHL_2,fb_RHL_1,fb_RHL_0;

////way-points
////					theta  0  1   2		 pos		 z  x  y
int FL_wp_0[3] = 	{0,-37,61},  //     [0  13 0]
		FL_wp_1[3] = 	{0,-88,101}, //     [0  9  -4]
		FL_wp_2[3] = 	{0,-71,109}, //     [0  9   0]
		
		FL_wp_3[3] = 	{0,-26,43},  //     [0  14 0]
		FL_wp_4[3] = 	{0,-46,59}, //      [0  13 -2]
		FL_wp_5[3] = 	{0,-48,51};	//      [0  13 -4]

		
//					theta  0  1   2		 pos		 z  x  y
int HL_wp_0[3] = 	{0,-37,61},  //     [0  13 0]
		HL_wp_1[3] = 	{0,-40,101}, //     [0  9  4]
		HL_wp_2[3] = 	{0,-71,109}, //     [0  9  0]
		
		HL_wp_3[3] = 	{0,-26,43},  //     [0  14 0]
		HL_wp_4[3] = 	{0,-27,59}, //      [0  13 2]
		HL_wp_5[3] = 	{0,-14,51};	//      [0  13 4]
		
//way-points
//					theta  0  1   2		 pos		 z  x  y
//int FL_wp_0[3] = 	{0,-37,61},  //     [0  13 0]
//		FL_wp_1[3] = 	{0,-83,106}, //     [0  9  -2.4]
//		FL_wp_2[3] = 	{0,-48,104}, //     [0  9   3]
//		
//		FL_wp_3[3] = 	{0,-16,47},  //     [0  13.5 3]
//		FL_wp_4[3] = 	{0,-25,58}, //      [0  13 2.4]
//		FL_wp_5[3] = 	{0,-28,59},	//      [0  13 1.8]
//		FL_wp_6[3] = 	{0,-32,60},  //     [0  13 1.2]
//		FL_wp_7[3] = 	{0,-35,61}, //      [0  13 0.6]
//		FL_wp_8[3] = 	{0,-37,61},	//      [0  13 0]
//		FL_wp_9[3] = 	{0,-40,61},  //     [0  13 -0.6]
//		FL_wp_10[3] = {0,-42,60}, //      [0  13 -1.2]
//		FL_wp_11[3] = {0,-44,60},	//      [0  13 -1.8]
//		FL_wp_12[3] = {0,-46,58};	//      [0  13 -2.4]
//		
//		
////					theta  0  1   2		 pos		 z  x  y
//int HL_wp_0[3] = 	{0,-37,61},  //     [0  13 0]
//		HL_wp_1[3] = 	{0,-43,102}, //     [0  9  3.6]
//		HL_wp_2[3] = 	{0,-80,108}, //     [0  9  -1.8]
//		
//		HL_wp_3[3] = 	{0,-38,51},  //     [0  13.5 -1.8]
//		HL_wp_4[3] = 	{0,-42,60}, //      [0  13 -1.2]
//		HL_wp_5[3] = 	{0,-40,61},	//      [0  13 -0.6]
//		HL_wp_6[3] = 	{0,-37,61},  //     [0  13 0]
//		HL_wp_7[3] = 	{0,-35,61}, //      [0  13 0.6]
//		HL_wp_8[3] = 	{0,-32,60},	//      [0  13 1.2]
//		HL_wp_9[3] = 	{0,-28,59},  //     [0  13 1.8]
//		HL_wp_10[3] = 	{0,-25,58}, //    [0  13 2.4]
//		HL_wp_11[3] = 	{0,-21,56},	//    [0  13 3]
//		HL_wp_12[3] = 	{0,-17,53};  //   [0  13 3.6]

		
/************************Subroutines_Prototype****************************/
void Delay(uint32_t nCount);// t = nCount (ms)
//void delay(uint16_t time); //delay func using systick
void Zero_state (void);
void X360_button (void);
	void action1 (void);
	void action2 (void);
	void action3 (void);
	void action4 (void);
void X360_Y1_Analog (void);
	void walk (void);


//Set
void LFL_2_set (int8_t phi, uint8_t k);
void LFL_1_set (int8_t phi, uint8_t k);
void LFL_0_set (int8_t phi, uint8_t k);

void LHL_2_set (int8_t phi, uint8_t k);
void LHL_1_set (int8_t phi, uint8_t k);
void LHL_0_set (int8_t phi, uint8_t k);

void RFL_2_set (int8_t phi, uint8_t k);
void RFL_1_set (int8_t phi, uint8_t k);
void RFL_0_set (int8_t phi, uint8_t k);

void RHL_2_set (int8_t phi, uint8_t k);
void RHL_1_set (int8_t phi, uint8_t k);
void RHL_0_set (int8_t phi, uint8_t k);

//Systick prototype
void feedback (uint16_t time);
void X360_X1_Analog (uint16_t time);
//Execute
void LFL_2_execute(uint16_t time);
void LFL_1_execute(uint16_t time);
void LFL_0_execute(uint16_t time);

void LHL_2_execute(uint16_t time);
void LHL_1_execute(uint16_t time);
void LHL_0_execute(uint16_t time);

void RFL_2_execute(uint16_t time);
void RFL_1_execute(uint16_t time);
void RFL_0_execute(uint16_t time);

void RHL_2_execute(uint16_t time);
void RHL_1_execute(uint16_t time);
void RHL_0_execute(uint16_t time);

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
		//X360_X1_Analog ();
	}
}

/************************Systick_interrupt_0.1ms******************************************/
void SysTick_Handler(void)
{
	SysTicker ++;
	//actuator
	LFL_2_execute(1);
	LFL_1_execute(1);
	LFL_0_execute(1);
	
	LHL_2_execute(1);
	LHL_1_execute(1);
	LHL_0_execute(1);
	
	RFL_2_execute(1);
	RFL_1_execute(1);
	RFL_0_execute(1);
	
	RHL_2_execute(1);
	RHL_1_execute(1);
	RHL_0_execute(1);
		
	//feedback
	feedback (10);
	X360_X1_Analog (10);
	//
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
		LFL_2_set (0,2);
		LHL_2_set (0,2);
		RFL_2_set (0,2);
		RHL_2_set (0,2);

		LFL_1_set (0,2);
		LHL_1_set (0,2);
		RFL_1_set (0,2);
		RHL_1_set (0,2);
		
		LFL_0_set (0,2);
		LHL_0_set (0,2);
		RFL_0_set (0,2);
		RHL_0_set (0,2);
}
void X360_button (void)
{
		switch (button)
					{
						case 1:action1();break;
						case 2:action2();break;
						case 4:action3();break;
						case 8:action4();break;
					}
}
	void action1 (void) //button B
	{	
		LFL_1_set (FL_wp_0[1],2);
		LFL_2_set (FL_wp_0[2],2);
		LHL_1_set (HL_wp_0[1],2);
		LHL_2_set (HL_wp_0[2],2);
		RFL_1_set (FL_wp_0[1],2);
		RFL_2_set (FL_wp_0[2],2);
		RHL_1_set (HL_wp_0[1],2);
		RHL_2_set (HL_wp_0[2],2);
		Delay (500);
	}

	void action2 (void) //button A
	{
//				LFL_1_set (FL_wp_3[1],1);
//				LFL_2_set (FL_wp_3[2],1);
//				LHL_1_set (HL_wp_6[1],1);
//				LHL_2_set (HL_wp_6[2],1);
//				RFL_1_set (FL_wp_9[1],1);
//				RFL_2_set (FL_wp_9[2],1);
//				RHL_1_set (HL_wp_12[1],1);
//				RHL_2_set (HL_wp_12[2],1);
//				Delay (200);
//		
//				LFL_1_set (FL_wp_4[1],1);
//				LFL_2_set (FL_wp_4[2],1);
//				LHL_1_set (HL_wp_7[1],1);
//				LHL_2_set (HL_wp_7[2],1);
//				RFL_1_set (FL_wp_10[1],1);
//				RFL_2_set (FL_wp_10[2],1);
//				RHL_1_set (HL_wp_1[1],1);
//				RHL_2_set (HL_wp_1[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_5[1],1);
//				LFL_2_set (FL_wp_5[2],1);
//				LHL_1_set (HL_wp_8[1],1);
//				LHL_2_set (HL_wp_8[2],1);
//				RFL_1_set (FL_wp_11[1],1);
//				RFL_2_set (FL_wp_11[2],1);
//				RHL_1_set (HL_wp_2[1],1);
//				RHL_2_set (HL_wp_2[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_6[1],1);
//				LFL_2_set (FL_wp_6[2],1);
//				LHL_1_set (HL_wp_9[1],1);
//				LHL_2_set (HL_wp_9[2],1);
//				RFL_1_set (FL_wp_12[1],1);
//				RFL_2_set (FL_wp_12[2],1);
//				RHL_1_set (HL_wp_3[1],1);
//				RHL_2_set (HL_wp_3[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_7[1],1);
//				LFL_2_set (FL_wp_7[2],1);
//				LHL_1_set (HL_wp_10[1],1);
//				LHL_2_set (HL_wp_10[2],1);
//				RFL_1_set (FL_wp_1[1],1);
//				RFL_2_set (FL_wp_1[2],1);
//				RHL_1_set (HL_wp_4[1],1);
//				RHL_2_set (HL_wp_4[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_8[1],1);
//				LFL_2_set (FL_wp_8[2],1);
//				LHL_1_set (HL_wp_11[1],1);
//				LHL_2_set (HL_wp_11[2],1);
//				RFL_1_set (FL_wp_2[1],1);
//				RFL_2_set (FL_wp_2[2],1);
//				RHL_1_set (HL_wp_5[1],1);
//				RHL_2_set (HL_wp_5[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_9[1],1);
//				LFL_2_set (FL_wp_9[2],1);
//				LHL_1_set (HL_wp_12[1],1);
//				LHL_2_set (HL_wp_12[2],1);
//				RFL_1_set (FL_wp_3[1],1);
//				RFL_2_set (FL_wp_3[2],1);
//				RHL_1_set (HL_wp_6[1],1);
//				RHL_2_set (HL_wp_6[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_10[1],1);
//				LFL_2_set (FL_wp_10[2],1);
//				LHL_1_set (HL_wp_1[1],1);
//				LHL_2_set (HL_wp_1[2],1);
//				RFL_1_set (FL_wp_4[1],1);
//				RFL_2_set (FL_wp_4[2],1);
//				RHL_1_set (HL_wp_7[1],1);
//				RHL_2_set (HL_wp_7[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_11[1],1);
//				LFL_2_set (FL_wp_11[2],1);
//				LHL_1_set (HL_wp_2[1],1);
//				LHL_2_set (HL_wp_2[2],1);
//				RFL_1_set (FL_wp_5[1],1);
//				RFL_2_set (FL_wp_5[2],1);
//				RHL_1_set (HL_wp_8[1],1);
//				RHL_2_set (HL_wp_8[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_12[1],1);
//				LFL_2_set (FL_wp_12[2],1);
//				LHL_1_set (HL_wp_3[1],1);
//				LHL_2_set (HL_wp_3[2],1);
//				RFL_1_set (FL_wp_6[1],1);
//				RFL_2_set (FL_wp_6[2],1);
//				RHL_1_set (HL_wp_9[1],1);
//				RHL_2_set (HL_wp_9[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_1[1],1);
//				LFL_2_set (FL_wp_1[2],1);
//				LHL_1_set (HL_wp_4[1],1);
//				LHL_2_set (HL_wp_4[2],1);
//				RFL_1_set (FL_wp_7[1],1);
//				RFL_2_set (FL_wp_7[2],1);
//				RHL_1_set (HL_wp_10[1],1);
//				RHL_2_set (HL_wp_10[2],1);
//				Delay (200);
//				
//				LFL_1_set (FL_wp_2[1],1);
//				LFL_2_set (FL_wp_2[2],1);
//				LHL_1_set (HL_wp_5[1],1);
//				LHL_2_set (HL_wp_5[2],1);
//				RFL_1_set (FL_wp_8[1],1);
//				RFL_2_set (FL_wp_8[2],1);
//				RHL_1_set (HL_wp_11[1],1);
//				RHL_2_set (HL_wp_11[2],1);
//				Delay (200);
	}

	void action3 (void) // button X
	{
		LFL_1_set (-26,2);
		LFL_2_set (99,2);
		LHL_1_set (-26,2);
		LHL_2_set (99,2);
		RFL_1_set (-26,2);
		RFL_2_set (99,2);
		RHL_1_set (-26,2);
		RHL_2_set (99,2);
		Delay (500);

	}
	
	void action4 (void) // button Y
	{

	}
void X360_Y1_Analog (void)
{
	if (Y1_Analog > 10)
	{
		walk();
	}		
}
	void walk (void)
	{
		while (Y1_Analog > 10)
		{
				
				LFL_1_set (FL_wp_5[1],1);
				LFL_2_set (FL_wp_5[2],1);
				RHL_1_set (HL_wp_5[1],1);
				RHL_2_set (HL_wp_5[2],1);
				RFL_1_set (FL_wp_3[1],1);
				RFL_2_set (FL_wp_3[2],1);
				LHL_1_set (HL_wp_3[1],1);
				LHL_2_set (HL_wp_3[2],1);
				Delay (100);
			
				LFL_1_set (FL_wp_1[1],1);
				LFL_2_set (FL_wp_1[2],1);
				RHL_1_set (HL_wp_1[1],1);
				RHL_2_set (HL_wp_1[2],1);
				RFL_1_set (FL_wp_3[1],1);
				RFL_2_set (FL_wp_3[2],1);
				LHL_1_set (HL_wp_3[1],1);
				LHL_2_set (HL_wp_3[2],1);
				Delay (100);
			
				LFL_1_set (FL_wp_2[1],1);
				LFL_2_set (FL_wp_2[2],1);
				RHL_1_set (HL_wp_2[1],1);
				RHL_2_set (HL_wp_2[2],1);
				RFL_1_set (FL_wp_4[1],1);
				RFL_2_set (FL_wp_4[2],1);
				LHL_1_set (HL_wp_4[1],1);
				LHL_2_set (HL_wp_4[2],1);
				Delay (100);
				
				LFL_1_set (FL_wp_3[1],1);
				LFL_2_set (FL_wp_3[2],1);
				RHL_1_set (HL_wp_3[1],1);
				RHL_2_set (HL_wp_3[2],1);
				RFL_1_set (FL_wp_5[1],1);
				RFL_2_set (FL_wp_5[2],1);
				LHL_1_set (HL_wp_5[1],1);
				LHL_2_set (HL_wp_5[2],1);
				Delay (100);
				
				LFL_1_set (FL_wp_3[1],1);
				LFL_2_set (FL_wp_3[2],1);
				RHL_1_set (HL_wp_3[1],1);
				RHL_2_set (HL_wp_3[2],1);
				RFL_1_set (FL_wp_1[1],1);
				RFL_2_set (FL_wp_1[2],1);
				LHL_1_set (HL_wp_1[1],1);
				LHL_2_set (HL_wp_1[2],1);
				Delay (100);
				
				LFL_1_set (FL_wp_4[1],1);
				LFL_2_set (FL_wp_4[2],1);
				RHL_1_set (HL_wp_4[1],1);
				RHL_2_set (HL_wp_4[2],1);
				RFL_1_set (FL_wp_2[1],1);
				RFL_2_set (FL_wp_2[2],1);
				LHL_1_set (HL_wp_2[1],1);
				LHL_2_set (HL_wp_2[2],1);
				Delay (100);
				
				LFL_1_set (FL_wp_5[1],1);
				LFL_2_set (FL_wp_5[2],1);
				RHL_1_set (HL_wp_5[1],1);
				RHL_2_set (HL_wp_5[2],1);
				RFL_1_set (FL_wp_3[1],1);
				RFL_2_set (FL_wp_3[2],1);
				LHL_1_set (HL_wp_3[1],1);
				LHL_2_set (HL_wp_3[2],1);
				Delay (100);
		}
}

// Systick subroutines
void feedback (uint16_t time)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		fb_LFL_2 = +((int16_t)TIM_GetCapture1(TIM5)-phi0_LFL_2)/angle_step;
		fb_LFL_1 = -((int16_t)TIM_GetCapture2(TIM5)-phi0_LFL_1)/angle_step;
		fb_LFL_0 = -((int16_t)TIM_GetCapture3(TIM5)-phi0_LFL_0)/angle_step;
		
		fb_LHL_2 = +((int16_t)TIM_GetCapture2(TIM2)-phi0_LHL_2)/angle_step;
		fb_LHL_1 = -((int16_t)TIM_GetCapture3(TIM2)-phi0_LHL_1)/angle_step;
		fb_LHL_0 = -((int16_t)TIM_GetCapture4(TIM2)-phi0_LHL_0)/angle_step;
		
		fb_RFL_2 = -((int16_t)TIM_GetCapture1(TIM3)-phi0_RFL_2)/angle_step;
		fb_RFL_1 = +((int16_t)TIM_GetCapture2(TIM3)-phi0_RFL_1)/angle_step;
		fb_RFL_0 = -((int16_t)TIM_GetCapture3(TIM3)-phi0_RFL_0)/angle_step;
		
		fb_RHL_2 = -((int16_t)TIM_GetCapture1(TIM4)-phi0_RHL_2)/angle_step;
		fb_RHL_1 = +((int16_t)TIM_GetCapture2(TIM4)-phi0_RHL_1)/angle_step;
		fb_RHL_0 = -((int16_t)TIM_GetCapture3(TIM4)-phi0_RHL_0)/angle_step;
	}
}

void X360_X1_Analog (uint16_t time)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
	}
}
           /************************ Set ****************************/
void LFL_2_set (int8_t phi, uint8_t k) //phi = angle (degree)
{
	phi_LFL_2= phi0_LFL_2 +phi*angle_step;
	k_LFL_2=k;
}
void LFL_1_set (int8_t phi, uint8_t k) 
{
	phi_LFL_1= phi0_LFL_1 -phi*angle_step;
	k_LFL_1=k;
}
void LFL_0_set (int8_t phi, uint8_t k) 
{
	phi_LFL_0= phi0_LFL_0 -phi*angle_step;
	k_LFL_0=k;
}

void LHL_2_set (int8_t phi, uint8_t k) //phi = angle (degree)
{
	phi_LHL_2= phi0_LHL_2 +phi*angle_step;
	k_LHL_2=k;
}
void LHL_1_set (int8_t phi, uint8_t k) 
{
	phi_LHL_1= phi0_LHL_1 -phi*angle_step;
	k_LHL_1=k;
}
void LHL_0_set (int8_t phi, uint8_t k) 
{
	phi_LHL_0= phi0_LHL_0 -phi*angle_step;
	k_LHL_0=k;
}

void RFL_2_set (int8_t phi, uint8_t k) //phi = angle (degree)
{
	phi_RFL_2= phi0_RFL_2 -phi*angle_step;
	k_RFL_2=k;
}
void RFL_1_set (int8_t phi, uint8_t k) 
{
	phi_RFL_1= phi0_RFL_1 +phi*angle_step;
	k_RFL_1=k;
}
void RFL_0_set (int8_t phi, uint8_t k) 
{
	phi_RFL_0= phi0_RFL_0 -phi*angle_step;
	k_RFL_0=k;
}

void RHL_2_set (int8_t phi, uint8_t k) //phi = angle (degree)
{
	phi_RHL_2= phi0_RHL_2 -phi*angle_step;
	k_RHL_2=k;
}
void RHL_1_set (int8_t phi, uint8_t k) 
{
	phi_RHL_1= phi0_RHL_1 +phi*angle_step;
	k_RHL_1=k;
}
void RHL_0_set (int8_t phi, uint8_t k) 
{
	phi_RHL_0= phi0_RHL_0 -phi*angle_step;
	k_RHL_0=k;
}

					/************************ Execute - Systicks ****************************/
//LFL
void LFL_2_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture1(TIM5);
		if (phi_LFL_2>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LFL_2) ==0)
				TIM_SetCompare1(TIM5, CCR_now);
		}
		else if (phi_LFL_2<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LFL_2) ==0)
				TIM_SetCompare1(TIM5, CCR_now);
		}
	}
}
void LFL_1_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture2(TIM5);
		if (phi_LFL_1>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LFL_1) ==0)
				TIM_SetCompare2(TIM5, CCR_now);
		}
		else if (phi_LFL_1<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LFL_1) ==0)
				TIM_SetCompare2(TIM5, CCR_now);
		}
	}
}
void LFL_0_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture3(TIM5);
		if (phi_LFL_0>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LFL_0) ==0)
				TIM_SetCompare3(TIM5, CCR_now);
		}
		else if (phi_LFL_0<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LFL_0) ==0)
				TIM_SetCompare3(TIM5, CCR_now);
		}
	}
}

//LHL
void LHL_2_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture2(TIM2);
		if (phi_LHL_2>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LHL_2) ==0)
				TIM_SetCompare2(TIM2, CCR_now);
		}
		else if (phi_LHL_2<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LHL_2) ==0)
				TIM_SetCompare2(TIM2, CCR_now);
		}
	}
}
void LHL_1_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture3(TIM2);
		if (phi_LHL_1>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LHL_1) ==0)
				TIM_SetCompare3(TIM2, CCR_now);
		}
		else if (phi_LHL_1<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LHL_1) ==0)
				TIM_SetCompare3(TIM2, CCR_now);
		}
	}
}
void LHL_0_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture4(TIM2);
		if (phi_LHL_0>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_LHL_0) ==0)
				TIM_SetCompare4(TIM2, CCR_now);
		}
		else if (phi_LHL_0<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_LHL_0) ==0)
				TIM_SetCompare4(TIM2, CCR_now);
		}
	}
}

//RFL
void RFL_2_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture1(TIM3);
		if (phi_RFL_2>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RFL_2) ==0)
				TIM_SetCompare1(TIM3, CCR_now);
		}
		else if (phi_RFL_2<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RFL_2) ==0)
				TIM_SetCompare1(TIM3, CCR_now);
		}
	}
}
void RFL_1_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture2(TIM3);
		if (phi_RFL_1>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RFL_1) ==0)
				TIM_SetCompare2(TIM3, CCR_now);
		}
		else if (phi_RFL_1<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RFL_1) ==0)
				TIM_SetCompare2(TIM3, CCR_now);
		}
	}
}
void RFL_0_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture3(TIM3);
		if (phi_RFL_0>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RFL_0) ==0)
				TIM_SetCompare3(TIM3, CCR_now);
		}
		else if (phi_RFL_0<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RFL_0) ==0)
				TIM_SetCompare3(TIM3, CCR_now);
		}
	}
}

//RHL
void RHL_2_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture1(TIM4);
		if (phi_RHL_2>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RHL_2) ==0)
				TIM_SetCompare1(TIM4, CCR_now);
		}
		else if (phi_RHL_2<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RHL_2) ==0)
				TIM_SetCompare1(TIM4, CCR_now);
		}
	}
}
void RHL_1_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture2(TIM4);
		if (phi_RHL_1>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RHL_1) ==0)
				TIM_SetCompare2(TIM4, CCR_now);
		}
		else if (phi_RHL_1<CCR_now)
		{
			CCR_now--;
			if ((SysTicker % k_RHL_1) ==0)
				TIM_SetCompare2(TIM4, CCR_now);
		}
	}
}
void RHL_0_execute(uint16_t time) // time = time*0.1 (ms)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		
		CCR_now=TIM_GetCapture3(TIM4);
		if (phi_RHL_0>CCR_now)
		{
			CCR_now++;
			if ((SysTicker % k_RHL_0) ==0)
				TIM_SetCompare3(TIM4, CCR_now);
		}
		else if (phi_RHL_0<CCR_now)
		{
			CCR_now--;                  
			if ((SysTicker % k_RHL_0) ==0)
				TIM_SetCompare3(TIM4, CCR_now);
		}
	}
}
