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
float pi= 3.141592654;

//LFL
#define phi0_LFL_2 2944
#define phi0_LFL_1 3880
#define phi0_LFL_0 6142

//LHL
#define phi0_LHL_2 7312
#define phi0_LHL_1 6064
#define phi0_LHL_0 4270

//RFL
#define phi0_RFL_2 7000
#define phi0_RFL_1 6220
#define phi0_RFL_0 4270

//RHL
#define phi0_RHL_2 3100
#define phi0_RHL_1 4036
#define phi0_RHL_0 5830


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

//Set angle
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

//Inverse Kinematics
#define a0    6
#define a1		6
#define a2		10
int8_t   theta_LFL_0,theta_LFL_1,theta_LFL_2,
				 theta_RFL_0,theta_RFL_1,theta_RFL_2,
				 theta_LHL_0,theta_LHL_1,theta_LHL_2,
				 theta_RHL_0,theta_RHL_1,theta_RHL_2;


unsigned int t=0;
//trotting elipse
float trot_a_L = 2,
			trot_b_L = 2;
float trot_a_R = 2,
			trot_b_R = 0;			

int8_t z_LF,y_LF,
			 z_RF,y_RF,
			 z_LH,y_LH,
			 z_RH,y_RH;
#define f 2 //(Hz)
#define sample_interval 1

////creep elipse
float a = 2,
			b = 2,
			z_step,y_step,
			z,y;
		
#define tdelay1 1000
#define tdelay2 500
#define speed1  4
#define speed2  2
		
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
		//gaits
		void creep (void);
//Inverse Kinematics
void LFL_IK (float x,float y,float z);
void RFL_IK (float x,float y,float z);	
void LHL_IK (float x,float y,float z);	
void RHL_IK (float x,float y,float z);	

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
void timer (uint16_t time);
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
	//timer
	timer (10);
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
		LFL_IK (8,0,14);
		RFL_IK (8,0,14);
		LHL_IK (8,0,14);
		RHL_IK (8,0,14);
		Delay (500);
	}

	void action2 (void) //button A
	{
		
	}

	void action3 (void) // button X
	{

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
			y_LF =   - trot_a_L*cos (2*pi*f*t/1000);
			z_LF =      trot_b_L*sin (2*pi*f*t/1000);
			z_LF = 14 - z_LF;
			LFL_IK (6,y_LF,z_LF);
			
			y_RH =    trot_a_R*cos (2*pi*f*t/1000);
			z_RH =    trot_b_R*sin (2*pi*f*t/1000);
			z_RH = 14 - z_RH;
			RHL_IK (6,y_RH,z_RH);
			
			
			y_RF =  - trot_a_R*cos (2*pi*f*t/1000 + pi);
			z_RF =    trot_b_R*sin (2*pi*f*t/1000 + pi);
			z_RF = 14 - z_RF;
			RFL_IK (6,y_RF,z_RF);
			
			y_LH =   trot_a_L*cos (2*pi*f*t/1000 + pi);
			z_LH =    trot_b_L*sin (2*pi*f*t/1000 + pi);
			z_LH = 14 - z_LH;
			LHL_IK (6,y_LH,z_LH);
			Delay (sample_interval);
		}
}
			void creep (void)
			{
				while (Y1_Analog > 10)
				{
					y =  a*cos (2*pi*t/100);
					z =  b*sin (2*pi*t/100);
					
					if (z < 0)
					{
						y =  a*cos (2*pi*t/100/3);
						z =  b*sin (2*pi*t/100/3);
					}
					
					z = 14 - z;
					y = 2 - y;
					
					LFL_IK (6,y,z);
					Delay (10);
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
		
		fb_LHL_2 = -((int16_t)TIM_GetCapture2(TIM2)-phi0_LHL_2)/angle_step;
		fb_LHL_1 = +((int16_t)TIM_GetCapture3(TIM2)-phi0_LHL_1)/angle_step;
		fb_LHL_0 = +((int16_t)TIM_GetCapture4(TIM2)-phi0_LHL_0)/angle_step;
		
		fb_RFL_2 = -((int16_t)TIM_GetCapture1(TIM3)-phi0_RFL_2)/angle_step;
		fb_RFL_1 = +((int16_t)TIM_GetCapture2(TIM3)-phi0_RFL_1)/angle_step;
		fb_RFL_0 = +((int16_t)TIM_GetCapture3(TIM3)-phi0_RFL_0)/angle_step;
		
		fb_RHL_2 = +((int16_t)TIM_GetCapture1(TIM4)-phi0_RHL_2)/angle_step;
		fb_RHL_1 = -((int16_t)TIM_GetCapture2(TIM4)-phi0_RHL_1)/angle_step;
		fb_RHL_0 = -((int16_t)TIM_GetCapture3(TIM4)-phi0_RHL_0)/angle_step;
	}
}
//timer
void timer (uint16_t time)
{
	static uint16_t ticker = 0;
	if ((uint16_t)(ticker+time)==GetTicker()) 
	{
		ticker = GetTicker();
		t=t+1;                               //t = 1ms
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
						/************************ Inverse Kinematics ****************************/
void LFL_IK (float x,float y,float z)
{
	float z1,
				theta_0_temp,theta_1_temp,theta_2_temp;
	
	z1 = sqrt(x*x + z*z - a0*a0);
	theta_0_temp = atan (z1/a0) + atan (x/z) - pi/2;
	theta_2_temp = acos (((y*y + z*z + x*x) - (a1*a1 + a2*a2 + a0*a0))/(2*a1*a2));
	theta_1_temp = atan (((a1+a2*cos(theta_2_temp))*y - a2* sin(theta_2_temp)*z1)/((a1+a2*cos(theta_2_temp))*z1 + a2* sin(theta_2_temp)*y));

	theta_LFL_0 =  (int8_t)(theta_0_temp*180/pi + 1/2);
	theta_LFL_1 =  (int8_t)(theta_1_temp*180/pi + 1/2);
	theta_LFL_2 =  (int8_t)(theta_2_temp*180/pi + 1/2);
	
	
	if (theta_LFL_0 < -40)  theta_LFL_0 = -40;
	if (theta_LFL_0 >  40)  theta_LFL_0 =  40;
	if (theta_LFL_1 < -80)  theta_LFL_0 = -80;
	if (theta_LFL_1 >  30)  theta_LFL_0 =  30;
	if (theta_LFL_2 <   0)  theta_LFL_0 =   0;
	if (theta_LFL_2 >  110) theta_LFL_0 = 110;
	
	LFL_0_set (theta_LFL_0,1);
	LFL_1_set (theta_LFL_1,1);
	LFL_2_set (theta_LFL_2,1);
}

void RFL_IK (float x,float y,float z)
{
	float z1,
				theta_0_temp,theta_1_temp,theta_2_temp;
	
	z1 = sqrt(x*x + z*z - a0*a0);
	theta_0_temp = atan (z1/a0) + atan (x/z) - pi/2;
	theta_2_temp = acos (((y*y + z*z + x*x) - (a1*a1 + a2*a2 + a0*a0))/(2*a1*a2));
	theta_1_temp = atan (((a1+a2*cos(theta_2_temp))*y - a2* sin(theta_2_temp)*z1)/((a1+a2*cos(theta_2_temp))*z1 + a2* sin(theta_2_temp)*y));

	theta_RFL_0 =  (int8_t)(theta_0_temp*180/pi + 1/2);
	theta_RFL_1 =  (int8_t)(theta_1_temp*180/pi + 1/2);
	theta_RFL_2 =  (int8_t)(theta_2_temp*180/pi + 1/2);
	
	
	if (theta_RFL_0 < -40)  theta_RFL_0 = -40;
	if (theta_RFL_0 >  40)  theta_RFL_0 =  40;
	if (theta_RFL_1 < -80)  theta_RFL_0 = -80;
	if (theta_RFL_1 >  30)  theta_RFL_0 =  30;
	if (theta_RFL_2 <   0)  theta_RFL_0 =   0;
	if (theta_RFL_2 >  110) theta_RFL_0 = 110;
	
	RFL_0_set (theta_RFL_0,1);
	RFL_1_set (theta_RFL_1,1);
	RFL_2_set (theta_RFL_2,1);
}

void LHL_IK (float x,float y,float z)
{
	float z1,
				theta_0_temp,theta_1_temp,theta_2_temp;
	
	z1 = sqrt(x*x + z*z - a0*a0);
	theta_0_temp = atan (z1/a0) + atan (x/z) - pi/2;
	theta_2_temp = acos (((y*y + z*z + x*x) - (a1*a1 + a2*a2 + a0*a0))/(2*a1*a2));
	theta_1_temp = atan (((a1+a2*cos(theta_2_temp))*y - a2* sin(theta_2_temp)*z1)/((a1+a2*cos(theta_2_temp))*z1 + a2* sin(theta_2_temp)*y));

	theta_LHL_0 =  (int8_t)(theta_0_temp*180/pi + 1/2);
	theta_LHL_1 =  (int8_t)(theta_1_temp*180/pi + 1/2);
	theta_LHL_2 =  (int8_t)(theta_2_temp*180/pi + 1/2);
	
	
	if (theta_LHL_0 < -40)  theta_LHL_0 = -40;
	if (theta_LHL_0 >  40)  theta_LHL_0 =  40;
	if (theta_LHL_1 < -80)  theta_LHL_0 = -80;
	if (theta_LHL_1 >  30)  theta_LHL_0 =  30;
	if (theta_LHL_2 <   0)  theta_LHL_0 =   0;
	if (theta_LHL_2 >  110) theta_LHL_0 = 110;
	
	LHL_0_set (theta_LHL_0,1);
	LHL_1_set (theta_LHL_1,1);
	LHL_2_set (theta_LHL_2,1);
}

void RHL_IK (float x,float y,float z)
{
	float z1,
				theta_0_temp,theta_1_temp,theta_2_temp;
	
	z1 = sqrt(x*x + z*z - a0*a0);
	theta_0_temp = atan (z1/a0) + atan (x/z) - pi/2;
	theta_2_temp = acos (((y*y + z*z + x*x) - (a1*a1 + a2*a2 + a0*a0))/(2*a1*a2));
	theta_1_temp = atan (((a1+a2*cos(theta_2_temp))*y - a2* sin(theta_2_temp)*z1)/((a1+a2*cos(theta_2_temp))*z1 + a2* sin(theta_2_temp)*y));

	theta_RHL_0 =  (int8_t)(theta_0_temp*180/pi + 1/2);
	theta_RHL_1 =  (int8_t)(theta_1_temp*180/pi + 1/2);
	theta_RHL_2 =  (int8_t)(theta_2_temp*180/pi + 1/2);
	
	
	if (theta_RHL_0 < -40)  theta_RHL_0 = -40;
	if (theta_RHL_0 >  40)  theta_RHL_0 =  40;
	if (theta_RHL_1 < -80)  theta_RHL_0 = -80;
	if (theta_RHL_1 >  30)  theta_RHL_0 =  30;
	if (theta_RHL_2 <   0)  theta_RHL_0 =   0;
	if (theta_RHL_2 >  110) theta_RHL_0 = 110;
	
	RHL_0_set (theta_RHL_0,1);
	RHL_1_set (theta_RHL_1,1);
	RHL_2_set (theta_RHL_2,1);
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
	phi_LHL_2= phi0_LHL_2 -phi*angle_step;
	k_LHL_2=k;
}
void LHL_1_set (int8_t phi, uint8_t k) 
{
	phi_LHL_1= phi0_LHL_1 +phi*angle_step;
	k_LHL_1=k;
}
void LHL_0_set (int8_t phi, uint8_t k) 
{
	phi_LHL_0= phi0_LHL_0 +phi*angle_step;
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
	phi_RFL_0= phi0_RFL_0 +phi*angle_step;
	k_RFL_0=k;
}

void RHL_2_set (int8_t phi, uint8_t k) //phi = angle (degree)
{
	phi_RHL_2= phi0_RHL_2 +phi*angle_step;
	k_RHL_2=k;
}
void RHL_1_set (int8_t phi, uint8_t k) 
{
	phi_RHL_1= phi0_RHL_1 -phi*angle_step;
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
