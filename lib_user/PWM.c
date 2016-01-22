/******************************************************************************************************

PA0 - TIM5CH1 - LFL_2 ,  PA1 - TIM5CH2 - LFL_1,  PA2 - TIM5CH3 - LFL_0
PB3 - TIM2CH2 - LHL_2,   PB10 - TIM2CH3 - LHL_1, PB11 - TIM2CH4 - LHL_0
PC6 - TIM3CH1 - RFL_2,   PC7 - TIM3CH2 - RFL_1,  PC8 - TIM3CH3 - RFL_0
PB6 - TIM4CH1 - RHL_2,   PB7 - TIM4CH2 - RHL_1,  PB8 - TIM4CH3 - RHL_0

******************************************************************************************************/
#include "stm32f4xx.h"
#include "PWM.h"

#define angle_step 38

//LFL
#define phi0_LFL_2 2944
#define phi0_LFL_1 3880
#define phi0_LFL_0 5050

//LHL
#define phi0_LHL_2 7312
#define phi0_LHL_1 6064
#define phi0_LHL_0 5050

//RFL
#define phi0_RFL_2 7000
#define phi0_RFL_1 6220
#define phi0_RFL_0 4738

//RHL
#define phi0_RHL_2 3100
#define phi0_RHL_1 4036
#define phi0_RHL_0 5284

void TIM_Config(void)
{
  /* Configure the TIM4 Output Channels */

	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  /* TIM2,3,4,5-APB1(42Mhz) clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOA,B,C,D clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* GPIOA Configuration: */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	/* GPIOB Configuration: */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8|GPIO_Pin_3|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	/* GPIOC Configuration: */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  
  /* Connect TIM5 pins to AF */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	
	/* Connect TIM2 pins to AF */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
	
	/* Connect TIM3 pins to AF */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3); 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	
	/* Connect TIM4 pins to AF*/  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
  
    /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535; //PWM = (42Mhz*2)/{(65535+1)*(24+1)*(0+1)}=50Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 24;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
}

void PWM_Config(void)
{
  /* Configure PWM mode for TIM4 Channels 1->4*/
	
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
  /*PWM Mode Init configuration*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	////////////////////////////////// TIM5////////////////////////////////
	//Init channel 1 
  TIM_OCInitStructure.TIM_Pulse = phi0_LFL_2; 
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	//Init channel 2
	TIM_OCInitStructure.TIM_Pulse = phi0_LFL_1; 
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	//Init channel 3
	TIM_OCInitStructure.TIM_Pulse = phi0_LFL_0; 
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM5, ENABLE);
  TIM_Cmd(TIM5, ENABLE); /* TIM5 enable counter */
	TIM_CtrlPWMOutputs(TIM5, ENABLE); /* TIM5 Main Output Enable */
	
	////////////////////////////////// TIM2////////////////////////////////
	//Init channel 2 
  TIM_OCInitStructure.TIM_Pulse = phi0_LHL_2; 
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	//Init channel 3
	TIM_OCInitStructure.TIM_Pulse = phi0_LHL_1; 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	//Init channel 4
	TIM_OCInitStructure.TIM_Pulse = phi0_LHL_0; 
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE); /* TIM2 enable counter */
	TIM_CtrlPWMOutputs(TIM2, ENABLE); /* TIM2 Main Output Enable */
	
	////////////////////////////////// TIM3////////////////////////////////
	//Init channel 1 
  TIM_OCInitStructure.TIM_Pulse = phi0_RFL_2;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//Init channel 2
	TIM_OCInitStructure.TIM_Pulse = phi0_RFL_1; 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//Init channel 3
	TIM_OCInitStructure.TIM_Pulse = phi0_RFL_0; 
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE); /* TIM3 enable counter */
	TIM_CtrlPWMOutputs(TIM3, ENABLE); /* TIM3 Main Output Enable */
	
	////////////////////////////////// TIM4////////////////////////////////
	//Init channel 1 
  TIM_OCInitStructure.TIM_Pulse = phi0_RHL_2; 
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	//Init channel 2
	TIM_OCInitStructure.TIM_Pulse = phi0_RHL_1; 
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	//Init channel 3
	TIM_OCInitStructure.TIM_Pulse = phi0_RHL_0; 
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE); /* TIM4 enable counter */
	TIM_CtrlPWMOutputs(TIM4, ENABLE); /* TIM4 Main Output Enable */
}

void PWMIN_Config(void)
{
  /* TIM5 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM5 CH2 pin (PA.01), 
     The Rising edge is used as active edge,
     The TIM5 CCR2 is used to compute the frequency value 
     The TIM5 CCR1 is used to compute the duty cycle value
  ------------------------------------------------------------ */
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* TIM5 & GPIOA clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* TIM4 chennel2 configuration : PA.01 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Connect TIM pin to AF2 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
  
  /* TIM4 configuration: PWM Input mode */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);

  /* Select the TIM5 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM5, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM5,TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM5, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
  
  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


