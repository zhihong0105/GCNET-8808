/*****************************************************************************
 * @Function : Platform specific code
 * @Author : Arthur
 * @Version : 1.5.4.0
 * @Modify Date : 20131016
 * @Copyright GridComm-PLC (C) 2013
 *****************************************************************************/

#include "platform.h"
#include "uart.h"
#include "gc2200.h"
#include "plcm.h"
#include "wireless.h"
#include "lora.h"
#include "global.h"
#include <stddef.h>

#ifdef SPI_LORA
uint8_t _ProductID[] = {0x88,0x8};		//Product ID
#else
uint8_t _ProductID[] = {0x88,0x0};		//Product ID
#endif

const uint8_t * GetProductID(void)
{
  return _ProductID;
}

static void readProtect()
{
	//Read protect
  if (FLASH_OB_GetRDP() == RESET)
  {
    DisableGlobalInterrupts();
    FLASH_Unlock();
    FLASH_OB_Unlock();
    FLASH_OB_RDPConfig(OB_RDP_Level_1);
    //FLASH_OB_Launch();
    FLASH_OB_Lock();
    FLASH_Lock();
    EnableGlobalInterrupts();
  }
}

uint8_t PLATFORM_SysInit()
{
  bool wirelessAble = false; //Assume no wireless capability
  /* Setup SysTick Timer for 1 msec interrupts  */
  SysTick_Config(SystemCoreClock/1000);

  //Turn on clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_SYSCFG|
      RCC_APB2Periph_DBGMCU,ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB,ENABLE);

  RCC_LSICmd(ENABLE);	//For watchdog

  readProtect();

  //GPIO configure
  //AF Outputs:
  //PA8 = MCO, PA7 = MOSI, PA5 = SCK, PA9 = TXD, PA2 = TX2
  //PB15 =  MOSI1, PB13 =  SCK1
  //PB3 = PWM_DAC
  //AF Inputs:
  //PA6 = MISO (No pull-up)
  //PA3 = RX2 (Pull-up)
  //PA10 = RXD
  //PB14 = MISO1
  //GPIO Outputs:
  //PA0 = RX_LED1, PA4 = SS, PA12 = 485C, PA15 = RST_RF
  //PB1 = RST, PB6 = RF_LED, PB7 = PLC_LED, PB8 = RX_LED, PB9 = TX_LED
  //PB12 = NSS1
  //GPIO Inputs:
  //PA1 = INT0, PA11 = DIO0
  //PB2 = DIO1, PB10=DIO2
  //PA9 (TX1), PA10 (RX1)

  GPIO_InitTypeDef gpioInit;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3|RCC_APB1Periph_WWDG,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

  //PA8=MCO,PA7=MOSI,PA5=SCK,PA9=TXD,PA2=TX2
  gpioInit.GPIO_Mode = GPIO_Mode_AF;
  gpioInit.GPIO_OType = GPIO_OType_PP;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_5|GPIO_Pin_9;

  GPIO_Init(GPIOA, &gpioInit);

  //PA6=MISO
  gpioInit.GPIO_Mode = GPIO_Mode_AF;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &gpioInit);

  //PA3=RX2,PA10=RXD
  gpioInit.GPIO_Mode = GPIO_Mode_AF;
  gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &gpioInit);

  //PA0=RX_LED1,PA4=SS,PA12=??,PA15=RST_RF
  gpioInit.GPIO_Mode = GPIO_Mode_OUT;
  gpioInit.GPIO_OType = GPIO_OType_PP;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_4|GPIO_Pin_12|GPIO_Pin_15;
  GPIO_Init(GPIOA, &gpioInit);

  //PB6=RF_LED,PB7=PLC_LED
  gpioInit.GPIO_Mode = GPIO_Mode_OUT;
  gpioInit.GPIO_OType = GPIO_OType_PP;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_Init(GPIOB, &gpioInit);

  //PB8=RX_LED,PB9=TX_LED
  gpioInit.GPIO_Mode = GPIO_Mode_IN;
  gpioInit.GPIO_OType = GPIO_OType_PP;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_Init(GPIOB, &gpioInit);


  //PB1=RST, pull up
  gpioInit.GPIO_Mode = GPIO_Mode_OUT;
  gpioInit.GPIO_OType = GPIO_OType_PP;
  gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOB, &gpioInit);

  //PA1=INT0,PA11=DIO0
  gpioInit.GPIO_Mode = GPIO_Mode_IN;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_11;
  GPIO_Init(GPIOA, &gpioInit);

  //PB2=DIO1,PB10=DIO2
  gpioInit.GPIO_Mode = GPIO_Mode_IN;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_10;
  GPIO_Init(GPIOB, &gpioInit);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);	//USART1
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);	//SPI1
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_0);	//MCO

  //Configure MCO pin, alternate function on PA8
  RCC_MCOConfig(RCC_MCOSource_HSE);

  //Configure SPI1
  SPI_InitTypeDef spiInit;

  spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spiInit.SPI_Mode = SPI_Mode_Master;
  spiInit.SPI_DataSize = SPI_DataSize_8b;
  spiInit.SPI_CPOL = SPI_CPOL_Low;
  spiInit.SPI_CPHA = SPI_CPHA_1Edge;
  spiInit.SPI_NSS = SPI_NSS_Soft;
  spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
  spiInit.SPI_CRCPolynomial = 7;

  SPI_Init(SPI1,&spiInit);
  SPI_RxFIFOThresholdConfig(SPI1,SPI_RxFIFOThreshold_QF);
  SPI_Cmd(SPI1,ENABLE);

//==============================================================================================
  //Init SPI2
  //PB12,PB13,PB14,PB15

  /* SPI2 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  gpioInit.GPIO_Mode = GPIO_Mode_OUT;
  gpioInit.GPIO_OType = GPIO_OType_PP;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
  gpioInit.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOB, &gpioInit);

  gpioInit.GPIO_Mode = GPIO_Mode_AF;
  gpioInit.GPIO_OType = GPIO_OType_PP;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
  gpioInit.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_Init(GPIOB, &gpioInit);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);	//SPI2
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_0);

  spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spiInit.SPI_Mode = SPI_Mode_Master;
  spiInit.SPI_DataSize = SPI_DataSize_8b;
  spiInit.SPI_CPOL = SPI_CPOL_Low;
  spiInit.SPI_CPHA = SPI_CPHA_1Edge;
  spiInit.SPI_NSS = SPI_NSS_Soft;
  spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
  spiInit.SPI_CRCPolynomial = 0;

  SPI_Init(SPI2,&spiInit);
  SPI_RxFIFOThresholdConfig(SPI2,SPI_RxFIFOThreshold_QF);
  SPI_Cmd(SPI2,ENABLE);

//==============================================================================================

  //Init TIM2 fow PWM
  //PB3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  gpioInit.GPIO_Mode = GPIO_Mode_AF;
  gpioInit.GPIO_OType = GPIO_OType_PP;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
  gpioInit.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOB, &gpioInit);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_2);	//TIM2 CH2

  PLATFORM_SetPWM(75L, 30); //75Hz, 30% duty cycle.

//==============================================================================================

  //Init PB4,PB5 for the relay
  gpioInit.GPIO_Mode = GPIO_Mode_OUT;
  gpioInit.GPIO_OType = GPIO_OType_PP;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpioInit.GPIO_Speed = GPIO_Speed_Level_2;
  gpioInit.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  GPIO_Init(GPIOB, &gpioInit);

//==============================================================================================

#ifdef SPI_LORA
  wirelessAble=true;
#endif

  __enable_irq();
  if (wirelessAble)
  {
    return (1 << WIRELESS_CAPABILITY_BIT);
  }
  else
  {
    return 0;
  }
}

void PLATFORM_InitWDOG()
{
#if ENABLE_WDOG
	IWDG_WriteAccessCmd(0x5555);
	IWDG_SetPrescaler(IWDG_Prescaler_256);	//clock at 40KHz/256= 156.25 Hz
	IWDG_SetReload(781);					//Watchdog period about 5 seconds
	//Wait for successful update of registers
//	while (IWDG_GetFlagStatus(IWDG_FLAG_PVU|IWDG_FLAG_RVU) != RESET);
	IWDG_ReloadCounter();
	IWDG_Enable();
#endif
}

void PLATFORM_InitTimer1()
{
	TIM_TimeBaseInitTypeDef timInit;
	timInit.TIM_ClockDivision = TIM_CKD_DIV1;
	timInit.TIM_CounterMode = TIM_CounterMode_Up;
	timInit.TIM_Prescaler = 48000;		//1 mS period @ 48MHz PCLK
	timInit.TIM_Period = 0xFFFFFFFF;

	TIM_TimeBaseInit(TIM3, &timInit);
	TIM_Cmd(TIM3, ENABLE);
	NVIC_ClearPendingIRQ(TIM3_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);            // Enable interrupt
	
}

void PLATFORM_InitExtIntr()
{
  EXTI_InitTypeDef extiInit;

  extiInit.EXTI_Line = EXTI_Line1;
  extiInit.EXTI_LineCmd = ENABLE;
  extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInit.EXTI_Trigger = EXTI_Trigger_Falling;

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
  EXTI_Init(&extiInit);

  extiInit.EXTI_Line = EXTI_Line11;
  extiInit.EXTI_LineCmd = ENABLE;
  extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInit.EXTI_Trigger = EXTI_Trigger_Rising;

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource11);
  EXTI_Init(&extiInit);

  NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
  NVIC_EnableIRQ(EXTI4_15_IRQn);            // Enable interrupt

  NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
  NVIC_EnableIRQ(EXTI0_1_IRQn);            // Enable interrupt
}

void EXTI4_15_IRQHandler(void)
{

  LED_RF_ON();
  EXTI_ClearFlag(EXTI_Line11);
#ifdef SPI_LORA
  SX1278_IntrHandler();
#endif
  LED_RF_OFF();

}


void EXTI0_1_IRQHandler(void)
{
  GC2200IntrHandler();
  EXTI_ClearFlag(EXTI_Line1);
}


void EXTI2_3_IRQHandler(void)
{

  LED_RF_ON();
  EXTI_ClearFlag(EXTI_Line2);
#ifdef SPI_LORA
  SX1278_IntrHandler();
#endif
  LED_RF_OFF();

}





static void (*_Timer0callback)() = NULL;

static uint32_t _ScheduleTime;


uint8_t PLATFORM_CheckScheduleTime(void)
{
	if(TIM_GetCounter(TIM3) > _ScheduleTime)
		return 1;
	else
		return 0;
	
}	

void PLATFORM_ScheduleInterrupt(int ticks, void (*callback)())
{
	
	TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
		_ScheduleTime=TIM_GetCounter(TIM3) + ticks;
    TIM_SetCompare1(TIM3, _ScheduleTime);
    _Timer0callback = callback;
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1))
	{
		TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
		TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
		if (_Timer0callback != NULL)
		{
			_Timer0callback();
		}
	#if DEBUG_LEVEL > 5
		printf("t%i\n",RTC->CNT);
	#endif
	}
}

uint32_t PLATFORM_GetTime(void)
{
  return TIM_GetCounter(TIM3);
}

void SysTick_Handler(void)
{
	MillisecondTick();
}

void USART2_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART2,USART_FLAG_RXNE) == SET) /* if no errors                      */
  {
    USART_ClearFlag(USART2,USART_FLAG_ORE);
    DisableGlobalInterrupts();
    PutUART2RxBuf(USART_ReceiveData(USART2));
    EnableGlobalInterrupts();
  }
  if (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == SET) /* if no errors                      */
  {
    WirelessTxInterrupt();
  }
}

void USART1_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET) /* if no errors                      */
  {
    USART_ClearFlag(USART1,USART_FLAG_ORE);
    DisableGlobalInterrupts();
    PutUART1RxBuf(USART_ReceiveData(USART1));
    EnableGlobalInterrupts();
  }
}

/*********************************************************************************************
 * @brief  send one byte to UART1 and wait for transmission complete
 * @param  byte to be sent
 * @return none
 *********************************************************************************************/
void PLATFORM_UART1TxByte(uint8_t txByte)
{
    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);	/* wait until transmit buffer available */
    USART_SendData(USART1,txByte);
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);	/* wait until transmission completed */
}

void PLATFORM_UART2TxByteNoWaitC(uint8_t txByte)
{
    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET); /* wait until transmit buffer available */
    USART_SendData(USART2,txByte);
}

/*********************************************************************************************
 * @brief  send one byte to UART1 but do not wait for transmission complete
 * @param  byte to be sent
 * @return none
 *********************************************************************************************/
void PLATFORM_UART1TxByteNoWaitC(uint8_t txByte)
{
    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);	/* wait until transmit buffer available */
    USART_SendData(USART1,txByte);
}

void PLATFORM_ConfigureUART(uint32_t baud, uint32_t frame)
{
	USART_InitTypeDef UARTinitDef;
	bool parity = true;	//assume parity

	UARTinitDef.USART_BaudRate = baud;
	UARTinitDef.USART_Mode = USART_CR1_RE | USART_CR1_TE;
	switch ((frame & 0x300) >> 8)
	{
	case 2:
		UARTinitDef.USART_Parity = USART_Parity_Even;
		break;
	case 3:
		UARTinitDef.USART_Parity = USART_Parity_Odd;
		break;
	default:
		UARTinitDef.USART_Parity = USART_Parity_No;
		parity = false;
		break;
	}
	if ((frame & 0x0f) == 4)
	{
		if (parity)
		{
			UARTinitDef.USART_WordLength = USART_WordLength_8b;
		}
		else
		{
			UARTinitDef.USART_WordLength = USART_WordLength_7b;
		}
	}
	else
	{
		if (parity)
		{
			UARTinitDef.USART_WordLength = USART_WordLength_9b;
		}
		else
		{
			UARTinitDef.USART_WordLength = USART_WordLength_8b;
		}
	}
	switch ((frame & 0x3000) >> 12)
	{
	case 2:
		UARTinitDef.USART_StopBits = USART_StopBits_1_5;
		break;
	case 3:
		UARTinitDef.USART_StopBits = USART_StopBits_2;
		break;
	default:
		UARTinitDef.USART_StopBits = USART_StopBits_1;
		break;
	}

	UARTinitDef.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &UARTinitDef);
	USART_Cmd(USART1, ENABLE);
	NVIC_ClearPendingIRQ(USART1_IRQn);         /* Enable uart RX interrupts */
	NVIC_EnableIRQ(USART1_IRQn);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void PLATFORM_ConfigureUART2(uint32_t baud, uint32_t frame)
{
  USART_InitTypeDef UARTinitDef;
  bool parity = true; //assume parity

  UARTinitDef.USART_BaudRate = baud;
  UARTinitDef.USART_Mode = USART_CR1_RE | USART_CR1_TE;
  switch ((frame & 0x300) >> 8)
  {
  case 2:
    UARTinitDef.USART_Parity = USART_Parity_Even;
    break;
  case 3:
    UARTinitDef.USART_Parity = USART_Parity_Odd;
    break;
  default:
    UARTinitDef.USART_Parity = USART_Parity_No;
    parity = false;
    break;
  }
  if ((frame & 0x0f) == 4)
  {
    if (parity)
    {
      UARTinitDef.USART_WordLength = USART_WordLength_8b;
    }
    else
    {
      UARTinitDef.USART_WordLength = USART_WordLength_7b;
    }
  }
  else
  {
    if (parity)
    {
      UARTinitDef.USART_WordLength = USART_WordLength_9b;
    }
    else
    {
      UARTinitDef.USART_WordLength = USART_WordLength_8b;
    }
  }
  switch ((frame & 0x3000) >> 12)
  {
  case 2:
    UARTinitDef.USART_StopBits = USART_StopBits_1_5;
    break;
  case 3:
    UARTinitDef.USART_StopBits = USART_StopBits_2;
    break;
  default:
    UARTinitDef.USART_StopBits = USART_StopBits_1;
    break;
  }

  UARTinitDef.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART2, &UARTinitDef);
  USART_Cmd(USART2, ENABLE);
  NVIC_ClearPendingIRQ(USART2_IRQn);         /* Enable uart RX interrupts */
  NVIC_EnableIRQ(USART2_IRQn);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

uint8_t PLATFORM_SPITransaction(uint8_t ch)
{
	while (SPI_GetTransmissionFIFOStatus(SPI1) != SPI_TransmissionFIFOStatus_Empty);
	SPI_SendData8(SPI1,ch);
	while (SPI_GetReceptionFIFOStatus(SPI1) == SPI_ReceptionFIFOStatus_Empty);
	return SPI_ReceiveData8(SPI1);
}

uint8_t PLATFORM_IsMediaIdle(void)
{
  return 1;
}	
	
void PLATFORM_SetPiority(void)
{
  NVIC_SetPriority(SysTick_IRQn, 0);
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_SetPriority(EXTI4_15_IRQn, 1);
  NVIC_SetPriority(EXTI0_1_IRQn, 1);
  NVIC_SetPriority(TIM3_IRQn, 1);
}	




void InitializeTimer2(uint32_t period);
void InitializePWMChannel2(uint32_t pulse);

void InitializeTimer2(uint32_t period)
{
  TIM_TimeBaseInitTypeDef timerInitStructure;
  TIM_Cmd(TIM2, DISABLE);
  timerInitStructure.TIM_Prescaler = 40;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = period;
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
  TIM_Cmd(TIM2, ENABLE);
}

void InitializePWMChannel2(uint32_t pulse)
{
  TIM_OCInitTypeDef outputChannelInit = {0,};
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
  outputChannelInit.TIM_Pulse = pulse;
  outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
  outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC2Init(TIM2, &outputChannelInit);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

}



//frequency is in Hz range from 50-100, and dutycycle is 0-100%
//if input freq>100, we assume the value will be in Khz.
void PLATFORM_SetPWM(uint32_t freq, uint8_t dutycycle)
{
  uint16_t period,pulse;
  uint32_t freq2;

  if(freq==0)
    freq2=1; //make sure the freq is not zero
  else if(freq<100)
    freq2=freq;
  else
    freq2=freq*1000;

  period=(uint16_t)(1000000L/freq2);
  pulse=period*dutycycle/100;
  InitializeTimer2(period);
  InitializePWMChannel2(pulse);

}



void PLATFORM_SetPowerRelay(uint8_t state)
{
  if(state)
  {
	GPIO_SetBits(GPIOB,GPIO_Pin_4);
	GPIO_SetBits(GPIOB,GPIO_Pin_5);
  }
  else
  {
	GPIO_ResetBits(GPIOB,GPIO_Pin_4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
  }
}




uint8_t _PWMfreq=70;
uint8_t _PWMdutycycle=50;

uint8_t PLATFORM_GetPWMfreq(void)
{
  return(_PWMfreq);
}

void PLATFORM_SetPWMfreq(uint8_t PWMfreq)
{
  _PWMfreq=PWMfreq;
  PLATFORM_SetPWM(PWMfreq,_PWMdutycycle);
}

uint8_t PLATFORM_GetPWMdutycycle(void)
{
  return(_PWMdutycycle);
}

void PLATFORM_SetPWMdutycycle(uint8_t PWMdutycycle)
{
  _PWMdutycycle=PWMdutycycle;
  PLATFORM_SetPWM(_PWMfreq,PWMdutycycle);
}



