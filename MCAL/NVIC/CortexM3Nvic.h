/*
 * CortexM3Nvic.h
 *
 *  Created on: Aug 30, 2024
 *      Author: moame
 */

#ifndef CORTEXM3NVIC_H_
#define CORTEXM3NVIC_H_

/********************************************Library Include Start********************************************/
#include"STD_TYPES.h"
#include"ErrType.h"
#include"BIT_MATH.h"

/********************************************Library Include End********************************************/

/**************************************NVIC Register Declaration Start**************************************/
#define NVIC_BASE			(0xE000E100)
#define NVIC	    		((Nvic_Reg*)NVIC_BASE)

typedef struct {
    volatile uint32 ISER[8];    // Interrupt Set-Enable Registers, offset: 0x00 - 0x1C
    uint32 RESERVED0[24];       // Reserved, offset: 0x20 - 0x7C
    volatile uint32 ICER[8];    // Interrupt Clear-Enable Registers, offset: 0x80 - 0x9C
    uint32 RESERVED1[24];       // Reserved, offset: 0xA0 - 0xFC
    volatile uint32 ISPR[8];    // Interrupt Set-Pending Registers, offset: 0x100 - 0x11C
    uint32 RESERVED2[24];       // Reserved, offset: 0x120 - 0x17C
    volatile uint32 ICPR[8];    // Interrupt Clear-Pending Registers, offset: 0x180 - 0x19C
    uint32 RESERVED3[24];       // Reserved, offset: 0x1A0 - 0x1FC
    volatile uint32 IABR[8];    // Interrupt Active Bit Registers, offset: 0x200 - 0x21C
    uint32 RESERVED4[56];       // Reserved, offset: 0x220 - 0x2FC
    volatile uint8  IP[240];    // Interrupt Priority Registers, offset: 0x300 - 0x3EF
    uint32 RESERVED5[644];      // Reserved, offset: 0x3F0 - 0xEFC
    volatile uint32 STIR;       // Software Trigger Interrupt Register, offset: 0xF00
}Nvic_Reg;
/**************************************NVIC Register Declaration end**************************************/

/*************************************NVIC DataType Declaration Start*************************************/

typedef enum {
    WWDG_IRQn                  = 0,  // Window Watchdog interrupt
    PVD_IRQn                   = 1,  // PVD through EXTI Line detection interrupt
    TAMPER_IRQn                = 2,  // Tamper interrupt
    RTC_IRQn                   = 3,  // RTC global interrupt
    FLASH_IRQn                 = 4,  // Flash global interrupt
    RCC_IRQn                   = 5,  // RCC global interrupt
    EXTI0_IRQn                 = 6,  // EXTI Line0 interrupt
    EXTI1_IRQn                 = 7,  // EXTI Line1 interrupt
    EXTI2_IRQn                 = 8,  // EXTI Line2 interrupt
    EXTI3_IRQn                 = 9,  // EXTI Line3 interrupt
    EXTI4_IRQn                 = 10, // EXTI Line4 interrupt
    DMA1_Channel1_IRQn         = 11, // DMA1 Channel 1 global interrupt
    DMA1_Channel2_IRQn         = 12, // DMA1 Channel 2 global interrupt
    DMA1_Channel3_IRQn         = 13, // DMA1 Channel 3 global interrupt
    DMA1_Channel4_IRQn         = 14, // DMA1 Channel 4 global interrupt
    DMA1_Channel5_IRQn         = 15, // DMA1 Channel 5 global interrupt
    DMA1_Channel6_IRQn         = 16, // DMA1 Channel 6 global interrupt
    DMA1_Channel7_IRQn         = 17, // DMA1 Channel 7 global interrupt
    ADC1_2_IRQn                = 18, // ADC1 and ADC2 global interrupt
    USB_HP_CAN_TX_IRQn         = 19, // USB High Priority or CAN TX interrupts
    USB_LP_CAN_RX0_IRQn        = 20, // USB Low Priority or CAN RX0 interrupts
    CAN_RX1_IRQn               = 21, // CAN RX1 interrupt
    CAN_SCE_IRQn               = 22, // CAN SCE interrupt
    EXTI9_5_IRQn               = 23, // EXTI Line[9:5] interrupts
    TIM1_BRK_IRQn              = 24, // TIM1 Break interrupt
    TIM1_UP_IRQn               = 25, // TIM1 Update interrupt
    TIM1_TRG_COM_IRQn          = 26, // TIM1 Trigger and Commutation interrupt
    TIM1_CC_IRQn               = 27, // TIM1 Capture Compare interrupt
    TIM2_IRQn                  = 28, // TIM2 global interrupt
    TIM3_IRQn                  = 29, // TIM3 global interrupt
    TIM4_IRQn                  = 30, // TIM4 global interrupt
    I2C1_EV_IRQn               = 31, // I2C1 event interrupt
    I2C1_ER_IRQn               = 32, // I2C1 error interrupt
    I2C2_EV_IRQn               = 33, // I2C2 event interrupt
    I2C2_ER_IRQn               = 34, // I2C2 error interrupt
    SPI1_IRQn                  = 35, // SPI1 global interrupt
    SPI2_IRQn                  = 36, // SPI2 global interrupt
    USART1_IRQn                = 37, // USART1 global interrupt
    USART2_IRQn                = 38, // USART2 global interrupt
    USART3_IRQn                = 39, // USART3 global interrupt
    EXTI15_10_IRQn             = 40, // EXTI Line[15:10] interrupts
    RTCAlarm_IRQn              = 41, // RTC Alarm through EXTI Line interrupt
    USBWakeUp_IRQn             = 42, // USB Wakeup from suspend through EXTI Line interrupt
    TIM8_BRK_IRQn              = 43, // TIM8 Break interrupt
    TIM8_UP_IRQn               = 44, // TIM8 Update interrupt
    TIM8_TRG_COM_IRQn          = 45, // TIM8 Trigger and Commutation interrupt
    TIM8_CC_IRQn               = 46, // TIM8 Capture Compare interrupt
    ADC3_IRQn                  = 47, // ADC3 global interrupt
    FSMC_IRQn                  = 48, // FSMC global interrupt
    SDIO_IRQn                  = 49, // SDIO global interrupt
    TIM5_IRQn                  = 50, // TIM5 global interrupt
    SPI3_IRQn                  = 51, // SPI3 global interrupt
    UART4_IRQn                 = 52, // UART4 global interrupt
    UART5_IRQn                 = 53, // UART5 global interrupt
    TIM6_IRQn                  = 54, // TIM6 global interrupt
    TIM7_IRQn                  = 55, // TIM7 global interrupt
    DMA2_Channel1_IRQn         = 56, // DMA2 Channel 1 global interrupt
    DMA2_Channel2_IRQn         = 57, // DMA2 Channel 2 global interrupt
    DMA2_Channel3_IRQn         = 58, // DMA2 Channel 3 global interrupt
    DMA2_Channel4_5_IRQn       = 59, // DMA2 Channel 4 and 5 global interrupt
} IRQn_Type;

typedef enum
{
	PriorityGroup1=3,
	PriorityGroup2=4,
	PriorityGroup3=5,
	PriorityGroup4=6,
	PriorityGroup5=7,
}IRQ_PriorityGroup;

/**************************************NVIC DataType Declaration End**************************************/

/*************************************NVIC Function Declaration Start*************************************/

uint8 Nvic_u8Enableinterrupt(IRQn_Type Copy_enuInterrupt);

uint8 Nvic_u8Disableinterrupt(IRQn_Type Copy_enuInterrupt);

uint8 Nvic_u8SetPendinginterrupt(IRQn_Type Copy_enuInterrupt);

uint8 Nvic_u8ClearPendinginterrupt(IRQn_Type Copy_enuInterrupt);

uint8 Nvic_u8GetActiveinterrupt(IRQn_Type Copy_enuInterrupt);

uint8 Nvic_u8SetinterruptPriority(IRQn_Type Copy_enuInterrupt,uint8 Copy_u8InterruptPriority);

uint8 NVIC_u8SetPriorityGroup(IRQ_PriorityGroup Copy_enuPriorityGroup);

/**************************************NVIC Function Declaration End**************************************/

/**************************************NVIC Macros Declaration Start**************************************/

#define Active							10u
#define NOT_ACTIVE						11u

#define FIRST_IRQN   					WWDG_IRQn
#define LAST_IRQN   					DMA2_Channel4_5_IRQn

#define NVIC_PRIORITY_POS					4u


/***************************************NVIC Macros Declaration End***************************************/

/**************************************SCB Register Declaration Start**************************************/

#define SCB_AIRCR				(*((volatile uint32*)0xE000E014))

/***************************************SCB Register Declaration End***************************************/

/**************************************SCB Macros Declaration Start**************************************/

#define VECTKEYSTAT						0x5FA
#define VECTKEYSTAT_POS					15u

#define AIRCR_MASK						(~((0xFFFF <<VECTKEY_POS)|(7u<<SCB_PRIORITY_POS)))
#define VECTKEY							0x5FA
#define VECTKEY_POS						16u

#define SCB_PRIORITY_POS					8u

/***************************************SCB Macros Declaration End***************************************/


#endif /* CORTEXM3NVIC_H_ */
