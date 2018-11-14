/*****************************************************************************
 * @Function : Platform header, Driver for PLC Module
 * @Author : Arthur, Welic
 * @Version : 1.5.2.0
 * @Modify Date : 20131016
 * @Copyright GridComm-PLC (C) 2013
 *****************************************************************************/

#ifndef PLATFORM_H_ //qinlimin
#define PLATFORM_H_

#include "stm32f0xx_conf.h"
#include "uart.h"
#include "global.h"

#include <stdint.h>
#include <stdbool.h>

uint8_t PLATFORM_SysInit(void);
void PLATFORM_ScheduleGC2200Rx(void);

#define SCAN_FREQ

// for log to flash level, 0 means do not log anything
#define LOG_TO_FLASH_LEVEL 					0
// max service nodes for stm32f071 will be ok
#define MAX_SERVICE_NODES			252
// max child nodes
#define MAX_CHILD_NODES				252

#define DEFAULT_MASTER1 {0x05,0x00,0x00,0x00}
#define DEFAULT_MASTER2 {0xff,0xff,0x00,0x00}
#define DEFAULT_MASTER3 {0x00,0x00,0x00,0x00}

//#define RS485_Tx_Enable   GPIO_SetBits(GPIOA,GPIO_Pin_12)
//#define RS485_Rx_Enable   GPIO_ResetBits(GPIOA,GPIO_Pin_12)

#define RS485_Tx_Enable
#define RS485_Rx_Enable

#define _SET_GPIO_PIN(x,y)		GPIO_SetBits(GPIO##x,GPIO_Pin_##y)
#define _CLR_GPIO_PIN(x,y)		GPIO_ResetBits(GPIO##x,GPIO_Pin_##y)
#define _READ_GPIO_PIN(x,y) 	GPIO_ReadInputDataBit(GPIO##x,GPIO_Pin_##y)

#define SET_GPIO_PIN(x)		_SET_GPIO_PIN(x)
#define CLR_GPIO_PIN(x)		_CLR_GPIO_PIN(x)
#define READ_GPIO_PIN(x)	_READ_GPIO_PIN(x)

#define SS_GPIO			A,4
//#define RESET_GPIO		B,13

#define RESET_GPIO		B,1

#define SS2_GPIO			B,12
#define RESET_RF		A,15

#define	SX1278_D0		A,11
#define SX1278_D1		B,2
#define SX1278_D2		B,10
#define INT0			A,1

#define RELAY1			B,4
#define RELAY2			B,5

//#define	LED1_ON()		GPIO_ResetBits(GPIOB,GPIO_Pin_15)
//#define	LED1_OFF()		GPIO_SetBits(GPIOB,GPIO_Pin_15)

//#define	LED1_ON()		GPIO_ResetBits(GPIOB,GPIO_Pin_7);GPIO_ResetBits(GPIOB,GPIO_Pin_4);GPIO_ResetBits(GPIOB,GPIO_Pin_5)
//#define	LED1_OFF()		GPIO_SetBits(GPIOB,GPIO_Pin_7);GPIO_SetBits(GPIOB,GPIO_Pin_4);GPIO_SetBits(GPIOB,GPIO_Pin_5)

#define	LED1_ON()		GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define	LED1_OFF()		GPIO_SetBits(GPIOB,GPIO_Pin_7)


#define LED_RX_ON() GPIO_SetBits(GPIOB,GPIO_Pin_8)
#define LED_RX_OFF() GPIO_ResetBits(GPIOB,GPIO_Pin_8)
#define LED_TX_ON() GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define LED_TX_OFF() GPIO_ResetBits(GPIOB,GPIO_Pin_9)

#define LED_RF_OFF() GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define LED_RF_ON() GPIO_SetBits(GPIOB,GPIO_Pin_6)

#define PIN_INT_TEST()  (READ_GPIO_PIN(SX1278_D0))
#define PIN_INT_TEST2()  (READ_GPIO_PIN(SX1278_D1))
#define PIN_INT_TEST3()  (READ_GPIO_PIN(SX1278_D2))
#define PIN_INT0() (READ_GPIO_PIN(INT0))


#define REBOOT()    NVIC_SystemReset()

//#define IsGC2200IntrPending()  NVIC_GetPendingIRQ(EXTI4_15_IRQn)
//#define SetGC2200PendingIntr()  NVIC_SetPendingIRQ(EXTI4_15_IRQn)
#define IsGC2200IntrPending()  NVIC_GetPendingIRQ(EXTI0_1_IRQn)
#define SetGC2200PendingIntr()  NVIC_SetPendingIRQ(EXTI0_1_IRQn)


/* Target UART defines */

#define _USART_FRAME_DATABITS_EIGHT           0x00000005UL                              /**< Mode EIGHT for USART_FRAME */
#define USART_FRAME_DATABITS_EIGHT            (_USART_FRAME_DATABITS_EIGHT << 0)        /**< Shifted mode EIGHT for USART_FRAME */
#define _USART_FRAME_STOPBITS_ONE             0x00000001UL                              /**< Mode ONE for USART_FRAME */
#define USART_FRAME_STOPBITS_ONE              (_USART_FRAME_STOPBITS_ONE << 12)         /**< Shifted mode ONE for USART_FRAME */
#define _USART_FRAME_PARITY_NONE              0x00000000UL                              /**< Mode NONE for USART_FRAME */
#define USART_FRAME_PARITY_NONE               (_USART_FRAME_PARITY_NONE << 8)           /**< Shifted mode NONE for USART_FRAME */
#define _USART_FRAME_PARITY_ODD               0x00000003UL                              /**< Mode ODD for USART_FRAME */
#define USART_FRAME_PARITY_ODD                (_USART_FRAME_PARITY_ODD << 8)            /**< Shifted mode ODD for USART_FRAME */
#define _USART_FRAME_PARITY_EVEN              0x00000002UL                              /**< Mode EVEN for USART_FRAME */
#define USART_FRAME_PARITY_EVEN               (_USART_FRAME_PARITY_EVEN << 8)           /**< Shifted mode EVEN for USART_FRAME */

#define UART1TXComplete		(USART1->ISR & USART_ISR_TC)

//For SM2200 Serial SPI
#define SS_CONNECTED
//#define BW_CONNECTED
//#define SHDN_CONNECTED

#define PLATFORM_DisableGlobalIRQ		__disable_irq
#define PLATFORM_EnableGlobalIRQ		__enable_irq

#define DisableUART2TxIntr()  USART_ITConfig(USART2, USART_IT_TXE, DISABLE)
#define EnableUART2TxIntr()	  USART_ITConfig(USART2, USART_IT_TXE, ENABLE)

//#define DisableGC2200Intr		NVIC_DisableIRQ(EXTI4_15_IRQn)
//#define EnableGC2200Intr		NVIC_EnableIRQ(EXTI4_15_IRQn)
#define DisableGC2200Intr		NVIC_DisableIRQ(EXTI0_1_IRQn);LED_TX_ON()
#define EnableGC2200Intr		NVIC_EnableIRQ(EXTI0_1_IRQn);LED_TX_OFF()

#define ClearPendingTimer1Intr	NVIC_ClearPendingIRQ(TIM3_IRQn)
#define EnableTimer1Intr				NVIC_EnableIRQ(TIM3_IRQn)
#define DisableTimer1Intr				NVIC_DisableIRQ(TIM3_IRQn)

#define EnableTimer2Intr				
#define DisableTimer21Intr			


#define ClearPendingTimer2Intr	
#define EnableTimer2Intr				
#define DisableTimer2Intr				


#define disable_systick_int  SysTick->CTRL  = 0
#define enable_systick_int   SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |\
                                              SysTick_CTRL_TICKINT_Msk   |\
                                              SysTick_CTRL_ENABLE_Msk
/* Enable SysTick IRQ and SysTick Timer */

static uint8_t inline uartChGet(void)
{
	while(UART1RxLen()==0);
	return ReadUART1();
}

//STM32 do not contain base node code
#define MASTER_CAPABLE
#define HAS_GCNET_MAIN			1

//#define TIME_MASK	0xffffff
#define TIME_MASK	0xffff

//#define UNCAL_TICK_FREQ	32768
#define UNCAL_TICK_FREQ	833

#define UARTCharGet(uartNo)			uartChGet()
#define UARTCharGetNonBlocking(uartNo)		ReadUART1()

#define ENABLE_WDOG 1

/* Target Watchdog */
void PLATFORM_InitWDOG(void);
void PLATFORM_InitTimer1(void);
void PLATFORM_InitExtIntr(void);

#if ENABLE_WDOG
#define PLATFORM_FeedWDOG	IWDG_ReloadCounter
#else
#define PLATFORM_FeedWDOG()
#endif
uint32_t PLATFORM_GetTime(void);
uint8_t PLATFORM_CheckScheduleTime(void);
void PLATFORM_ScheduleInterrupt(int ticks, void (*callback)());

void PLATFORM_ConfigureUART(uint32_t baud, uint32_t frame);
void PLATFORM_ConfigureUART2(uint32_t baud, uint32_t frame);
void PLATFORM_UART1TxByte(uint8_t txByte);				//send one byte to uart1
void PLATFORM_UART2TxByteNoWaitC(uint8_t txByte);
void PLATFORM_UART1TxByteNoWaitC(uint8_t txByte);
uint8_t PLATFORM_SPITransaction(uint8_t ch);

#define FLASH_PAGE_SIZE           2048
#define TEMP_UPGRADE_IMAGE_PAGES  24
#define IMAGE_START_ADDR        0x08000000
#define TEMP_UPGRADE_IMAGE_ADDR 0x0800C000
#define FLASH_SYSTEM_CFG     	  0x08018000  	 //system configuration
#define FLASH_SYSTEM_CFG2    	  0x08018800  	 //duplicate copy of system configuration
#define FLASH_ENCR_KEYS		      0x08019000

#define PLATFORM_InitFlash()
#define PLATFORM_DeInitFlash()

void PLATFORM_EraseFlashPage(uint32_t startAddress);
void PLATFORM_WriteFlash(uint32_t *address, uint8_t *pdata, int numBytes);
void PLATFORM_CompleteFirmwareUpgrade(uint8_t numPages, bool newFwImage);
uint8_t PLATFORM_IsMediaIdle(void);

const uint8_t * GetProductID(void);

void PLATFORM_SetPiority(void);

void PLATFORM_SetPWM(uint32_t freq, uint8_t dutycycle);

uint8_t PLATFORM_GetPWMdutycycle(void);
void PLATFORM_SetPWMdutycycle(uint8_t PWMdutycycle);
uint8_t PLATFORM_GetPWMfreq(void);
void PLATFORM_SetPWMfreq(uint8_t PWMfreq);

void PLATFORM_SetPowerRelay(uint8_t state);

#define SPI_LORA

#define GC8808

#endif /* PLATFORM_H_ */
