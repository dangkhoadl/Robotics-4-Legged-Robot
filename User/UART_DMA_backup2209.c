/* ====================================================================================
File name:      USER_UART.C                                     
Originator:	    Robotechco &  Luffy D.Monkey 
Description: 	Functions for configuring and sending data to PC through UART1.
Target: 	    STM32F407

        - BaudRate = input of functions
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and Transmit enabled
--------------------------------------------------------------------------------------*/
/* Includes --------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "UART_DMA.h"
/* Private typedef -------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------*/



/*UART1 configuration with DMA----------------------------------------------------------------------*/
void UART1_DMA_CONFIG(uint8_t *TxAddr,uint8_t Tx_size,uint8_t *RxAddr, uint8_t Rx_size, uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  DMA_InitTypeDef   DMA_InitStructure;
  
  //Peripheral Clock Enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  
  /*-----------------------USART1 GPIO configuration----------------------------*/
  //Connect USART pins to AF (TX:PA9) (RX:PA10)
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  
  //Configure USART Tx and Rx as alternate function push-pull (TX:PA9) (RX:PA10)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9| GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*-----------------------USART1 configuration ------------------------------*/
  //Enable the USART OverSampling by 8
  USART_OverSampling8Cmd(USART1, ENABLE); 
  
  /* USART1 configured as follow:
        - BaudRate = input of functions
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and Transmit enabled
  */
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  //When using Parity the word length must be configured to 9 bits
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  /*-----Configure DMA controller to manage USART TX and RX DMA request ------*/ 
  //Configure DMA Initialization Structure
  DMA_DeInit(DMA2_Stream7);//reset to default
	DMA_DeInit(DMA2_Stream2);
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART1->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  //Configure TX DMA
  DMA_InitStructure.DMA_BufferSize = Tx_size;
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;//according to reference manual STM32F407 =.=
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)TxAddr ;
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);
  //Configure RX DMA
  DMA_InitStructure.DMA_BufferSize = Rx_size;
	DMA_InitStructure.DMA_Channel = DMA_Channel_4; //according to reference manual STM32F407 =.=
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)RxAddr ; 
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);

  //Enable USART
  USART_Cmd(USART1, ENABLE);
}/*------------------------------------End of UART1_DMA_CONFIG functions----------------------------*/

void USART_config (void)
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	/* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  /* Enable the Receive interrupt */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);
}
/****************************************************************************************************/
void DMA_TX(void)
{
	//Enable DMA Stream Tx & Enable USART DMA TX Requsts
  DMA_Cmd(DMA2_Stream7, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  
  //Waiting the end of Data transfer
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);    
  while (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7)==RESET);
  
  //Disable DMA Stream Tx
  DMA_Cmd(DMA2_Stream7, DISABLE);

  //Clear DMA Transfer Complete Flags
  DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
  //Clear USART Transfer Complete Flags
  USART_ClearFlag(USART1, USART_FLAG_TC);
}/*-----------------------------------------End of DMA_TX functions---------------------------------*/
/****************************************************************************************************/

/****************************************************************************************************/
void SENDDATA(float Value, uint8_t* TxAddr, uint8_t channel)
{
  uint8_t* Data = (uint8_t*)&Value;
  TxAddr[0] = 0xFF;
  switch (channel) {
    case 0: TxAddr[1] = 0xF0; break;
    case 1: TxAddr[1] = 0xF1; break;
    case 2: TxAddr[1] = 0xF2; break;
    case 3: TxAddr[1] = 0xF3; break;
		case 4: TxAddr[1] = 0xF4; break;
		case 5: TxAddr[1] = 0xF5; break;
		case 6: TxAddr[1] = 0xF6; break;
  }
  TxAddr[2] = Data[0];
  TxAddr[3] = Data[1];
  TxAddr[4] = Data[2];
  TxAddr[5] = Data[3];
  TxAddr[6] = 0xFE;
  DMA_TX();
}/*--------------------------------End of SENDDATA functions----------------------------------------*/


/****************************************************************************************************/
/******************* (C) COPYRIGHT 2013 *****END OF FILE*****************************/
