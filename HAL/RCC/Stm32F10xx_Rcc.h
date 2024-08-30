/*
 * Stm32F10xx_Rcc.h
 *
 *  Created on: Aug 29, 2024
 *      Author: moame
 */

#ifndef STM32F10XX_RCC_H_
#define STM32F10XX_RCC_H_

/********************************************Library Include Start********************************************/
#include"STD_TYPES.h"
#include"ErrType.h"
#include"BIT_MATH.h"
/********************************************Library Include End********************************************/

/**************************************RCC Macros Declaration Start**************************************/


#define PLLMUL_POS								18u
#define PLLMUL_MASK								(~(0xF<<PLLMUL_POS))

#define SystemClock_POS							2u
#define SystemClock_MASK						(~(0<<SystemClock_POS))

#define APB1_POS								8u
#define APB2_POS								11u
#define AHB_POS									4u

#define APB1_MASK								(~(7<<APB1_POS))
#define APB2_MASK								(~(7<<APB2_POS))
#define AHB_MASK								(~(0xF<<AHB_POS))

#define IsAPB1									30u
#define IsAPB2									31u
/***************************************RCC Macros Declaration End***************************************/

/**************************************RCC Register Declaration Start**************************************/
// Define the base address for RCC
#define RCC_BASE        0x40021000

// Define the RCC registers structure
typedef struct {
    volatile uint32 CR;         // Clock control register
    volatile uint32 CFGR;       // Clock configuration register
    volatile uint32 CIR;        // Clock interrupt register
    volatile uint32 APB2RSTR;   // APB2 peripheral reset register
    volatile uint32 APB1RSTR;   // APB1 peripheral reset register
    volatile uint32 AHBENR;     // AHB peripheral clock enable register
    volatile uint32 APB2ENR;    // APB2 peripheral clock enable register
    volatile uint32 APB1ENR;    // APB1 peripheral clock enable register
    volatile uint32 BDCR;       // Backup domain control register
    volatile uint32 CSR;        // Control/status register
} RCC_TypeDef;

// Create an instance of the RCC structure
#define RCC ((RCC_TypeDef *) RCC_BASE)

#define PLLSRC			16u
#define PLLXTPRE		17u

#define PLLRDY			25u
#define PLLON			24u

#define HSEON			16u
#define HSERDY			17u
#define HSEBYP			18u

#define HSION			0u
#define HSIRDY			1u

#define LSION			0u
#define LSIRDY			1u

#define LSEON			0u
#define LSERDY			1u
#define LSEBYP			2u

#define LSION			0u
#define LSIRDY			1u
/**************************************RCC Register Declaration end**************************************/

/*************************************RCC DataType Declaration Start*************************************/

// Define an enumeration for PLL multiplier values
typedef enum {
    PLL_MUL_2 = 0b0000,    // PLL input clock x 2
    PLL_MUL_3 = 0b0001,    // PLL input clock x 3
    PLL_MUL_4 = 0b0010,    // PLL input clock x 4
    PLL_MUL_5 = 0b0011,    // PLL input clock x 5
    PLL_MUL_6 = 0b0100,    // PLL input clock x 6
    PLL_MUL_7 = 0b0101,    // PLL input clock x 7
    PLL_MUL_8 = 0b0110,    // PLL input clock x 8
    PLL_MUL_9 = 0b0111,    // PLL input clock x 9
    PLL_MUL_10 = 0b1000,   // PLL input clock x 10
    PLL_MUL_11 = 0b1001,   // PLL input clock x 11
    PLL_MUL_12 = 0b1010,   // PLL input clock x 12
    PLL_MUL_13 = 0b1011,   // PLL input clock x 13
    PLL_MUL_14 = 0b1100,   // PLL input clock x 14
    PLL_MUL_15 = 0b1101,   // PLL input clock x 15
    PLL_MUL_16 = 0b1110,   // PLL input clock x 16
} PLL_MUL_Factor;

#define HSEClockNotDivided						0u
#define HSEClockDividedBy2						1u

#define HSIOscillatorClockDividedBy2			0u
#define HSEOscillatorClock						1u

typedef struct
{
	uint8 HSEDivider;
	uint8 PLLEntryClockSource;
	PLL_MUL_Factor PLLMultiplicationFactor;
}PLLConfig;

typedef enum
{
	HSI=0,
	HSE=1,
	PLL=2,
}Clock;

typedef enum {
    APB_PRESCALER_1 = 0b000,  // No division
    APB_PRESCALER_2 = 0b100,  // Divide by 2
    APB_PRESCALER_4 = 0b101,  // Divide by 4
    APB_PRESCALER_8 = 0b110,  // Divide by 8
    APB_PRESCALER_16 = 0b111  // Divide by 16
} APB_Prescaler;

typedef enum {
    AHB_PRESCALER_1 = 0b0000,   // No division
    AHB_PRESCALER_2 = 0b1000,   // Divide by 2
    AHB_PRESCALER_4 = 0b1001,   // Divide by 4
    AHB_PRESCALER_8 = 0b1010,   // Divide by 8
    AHB_PRESCALER_16 = 0b1011,  // Divide by 16
    AHB_PRESCALER_64 = 0b1100,  // Divide by 64
    AHB_PRESCALER_128 = 0b1101, // Divide by 128
    AHB_PRESCALER_256 = 0b1110, // Divide by 256
    AHB_PRESCALER_512 = 0b1111  // Divide by 512
} AHB_Prescaler;

typedef enum {
    // AHB Peripherals
    DMA1    = (1 << 0),  // DMA1 Clock Enable
    DMA2    = (1 << 1),  // DMA2 Clock Enable
    SRAM    = (1 << 2),  // SRAM Interface Clock Enable
    FLITF   = (1 << 4),  // FLITF Clock Enable
    CRC     = (1 << 6),  // CRC Clock Enable
    FSMC    = (1 << 8),  // FSMC Clock Enable
    SDIO    = (1 << 10), // SDIO Clock Enable

    // APB1 Peripherals
    TIM2    = (1 << 0)|(1<<IsAPB1),  // TIM2 Clock Enable
    TIM3    = (1 << 1)|(1<<IsAPB1),  // TIM3 Clock Enable
    TIM4    = (1 << 2)|(1<<IsAPB1),  // TIM4 Clock Enable
    TIM5    = (1 << 3)|(1<<IsAPB1),  // TIM5 Clock Enable
    TIM6    = (1 << 4)|(1<<IsAPB1),  // TIM6 Clock Enable
    TIM7    = (1 << 5)|(1<<IsAPB1),  // TIM7 Clock Enable
    WWDG    = (1 << 11)|(1<<IsAPB1), // Window Watchdog Clock Enable
    SPI2    = (1 << 14)|(1<<IsAPB1), // SPI2 Clock Enable
    SPI3    = (1 << 15)|(1<<IsAPB1), // SPI3 Clock Enable
    USART2  = (1 << 17)|(1<<IsAPB1), // USART2 Clock Enable
    USART3  = (1 << 18)|(1<<IsAPB1), // USART3 Clock Enable
    UART4   = (1 << 19)|(1<<IsAPB1), // UART4 Clock Enable
    UART5   = (1 << 20)|(1<<IsAPB1), // UART5 Clock Enable
    I2C1    = (1 << 21)|(1<<IsAPB1), // I2C1 Clock Enable
    I2C2    = (1 << 22)|(1<<IsAPB1), // I2C2 Clock Enable
    USB     = (1 << 23)|(1<<IsAPB1), // USB Clock Enable
    CAN1    = (1 << 25)|(1<<IsAPB1), // CAN1 Clock Enable
    CAN2    = (1 << 26)|(1<<IsAPB1), // CAN2 Clock Enable
    BKP     = (1 << 27)|(1<<IsAPB1), // Backup Interface Clock Enable
    PWR     = (1 << 28)|(1<<IsAPB1), // Power Interface Clock Enable
    DAC     = (1 << 29)|(1<<IsAPB1), // DAC Clock Enable

    // APB2 Peripherals
    AFIO    = (1 << 0)|(1<<IsAPB2),  // Alternate Function I/O Clock Enable
    GPIOA   = (1 << 2)|(1<<IsAPB2),  // GPIOA Clock Enable
    GPIOB   = (1 << 3)|(1<<IsAPB2),  // GPIOB Clock Enable
    GPIOC   = (1 << 4)|(1<<IsAPB2),  // GPIOC Clock Enable
    GPIOD   = (1 << 5)|(1<<IsAPB2),  // GPIOD Clock Enable
    GPIOE   = (1 << 6)|(1<<IsAPB2),  // GPIOE Clock Enable
    GPIOF   = (1 << 7)|(1<<IsAPB2),  // GPIOF Clock Enable
    GPIOG   = (1 << 8)|(1<<IsAPB2),  // GPIOG Clock Enable
    ADC1    = (1 << 9)|(1<<IsAPB2),  // ADC1 Clock Enable
    ADC2    = (1 << 10)|(1<<IsAPB2), // ADC2 Clock Enable
    TIM1    = (1 << 11)|(1<<IsAPB2), // TIM1 Clock Enable
    SPI1    = (1 << 12)|(1<<IsAPB2), // SPI1 Clock Enable
} PeripheralEnable;

/**************************************RCC DataType Declaration End**************************************/

/*************************************RCC Function Declaration Start*************************************/

uint8 RCC_u8PLLConfig(PLLConfig Copy_stConfig);
void RCC_vEnablePLL();
void RCC_vDisablePLL();
void RCC_vEnableHSE();
void RCC_vDisableHSE();
void RCC_vEnableHSI();
void RCC_vDisableHSI();
void RCC_vEnableLSE();
void RCC_vDisableLSE();
void RCC_vEnableLSI();
void RCC_vDisableLSI();
uint8 RCC_u8SelcetSystemClock(Clock Copy_enSystemClock);
uint8 RCC_u8SetAPB1Prescaler(APB_Prescaler Copy_enPrescaler);
uint8 RCC_u8SetAPB2Prescaler(APB_Prescaler Copy_enPrescaler);
uint8 RCC_u8SetAHBPrescaler(AHB_Prescaler Copy_enPrescaler);
void RCC_vEnablePeripheralClock(PeripheralEnable Copy_enPeriphral);
void RCC_vDisablePeripheralClock(PeripheralEnable Copy_enPeriphral);

/**************************************RCC Function Declaration End**************************************/



#endif /* STM32F10XX_RCC_H_ */
