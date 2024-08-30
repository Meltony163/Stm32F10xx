#include "Stm32F10xx_Rcc.h"
#include "STD_TYPES.h"
#include "ErrType.h"

/**
 * @brief Configures the PLL (Phase-Locked Loop) with the specified settings.
 *
 * @param Copy_stConfig A structure containing PLL configuration parameters:
 *        - HSEDivider: Divider for HSE clock (e.g., not divided or divided by 2)
 *        - PLLEntryClockSource: Clock source for PLL (HSI or HSE)
 *        - PLLMultiplicationFactor: Multiplication factor for PLL
 *
 * @return uint8 Returns OK (0) if configuration is successful, NOK (1) otherwise.
 */
uint8 RCC_u8PLLConfig(PLLConfig Copy_stConfig)
{
	uint8 Local_u8ErrState = OK;

	// Disable PLL
	CLR_BIT(RCC->CR, PLLON);

	// Set PLL source
	if (Copy_stConfig.PLLEntryClockSource == HSIOscillatorClockDividedBy2)
	{
		CLR_BIT(RCC->CFGR, PLLSRC);
	}
	else if (Copy_stConfig.PLLEntryClockSource == HSEOscillatorClock)
	{
		if (Copy_stConfig.HSEDivider == HSEClockDividedBy2)
		{
			SET_BIT(RCC->CFGR, PLLXTPRE);
			SET_BIT(RCC->CFGR, PLLSRC);
		}
		else if (Copy_stConfig.HSEDivider == HSEClockNotDivided)
		{
			CLR_BIT(RCC->CFGR, PLLXTPRE);
			SET_BIT(RCC->CFGR, PLLSRC);
		}
		else
		{
			Local_u8ErrState = NOK; // Invalid HSE Divider
		}
	}

	// Set PLL multiplication factor
	RCC->CFGR = ((RCC->CFGR) & PLLMUL_MASK) | (Copy_stConfig.PLLMultiplicationFactor << PLLMUL_POS);

	// Enable PLL and wait for it to be ready
	SET_BIT(RCC->CR, PLLON);
	while (GET_BIT(RCC->CR, PLLRDY) == 0);

	return Local_u8ErrState;
}

/**
 * @brief Selects the system clock source.
 *
 * @param Copy_enSystemClock The clock source to be selected (HSI, HSE, or PLL)
 *
 * @return uint8 Returns OK (0) if the system clock is selected successfully, NOK (1) otherwise.
 */
uint8 RCC_u8SelcetSystemClock(Clock Copy_enSystemClock)
{
	uint8 Local_u8ErrState = OK;

	if (Copy_enSystemClock == HSE)
	{
		RCC->CFGR = ((RCC->CFGR) & SystemClock_MASK) | (Copy_enSystemClock << SystemClock_POS);
		RCC_vEnableHSE();
	}
	else if (Copy_enSystemClock == HSI)
	{
		RCC->CFGR = ((RCC->CFGR) & SystemClock_MASK) | (Copy_enSystemClock << SystemClock_POS);
		RCC_vEnableHSI();
	}
	else if (Copy_enSystemClock == PLL)
	{
		RCC->CFGR = ((RCC->CFGR) & SystemClock_MASK) | (Copy_enSystemClock << SystemClock_POS);
		RCC_vEnablePLL();
	}
	else
	{
		Local_u8ErrState = NOK; // Invalid clock source
	}

	return Local_u8ErrState;
}

/**
 * @brief Enables the PLL (Phase-Locked Loop).
 *
 * This function will block until the PLL is ready.
 */
void RCC_vEnablePLL()
{
	SET_BIT(RCC->CR, PLLON);
	while (GET_BIT(RCC->CR, PLLRDY) == 0);
}

/**
 * @brief Disables the PLL (Phase-Locked Loop).
 */
void RCC_vDisablePLL()
{
	CLR_BIT(RCC->CR, PLLON);
}

/**
 * @brief Enables the HSE (High-Speed External) oscillator.
 *
 * This function will block until the HSE is ready.
 */
void RCC_vEnableHSE()
{
	SET_BIT(RCC->CR, HSEON);
	while (GET_BIT(RCC->CR, HSERDY) == 0);
}

/**
 * @brief Disables the HSE (High-Speed External) oscillator.
 */
void RCC_vDisableHSE()
{
	CLR_BIT(RCC->CR, HSEON);
}

/**
 * @brief Enables the HSI (High-Speed Internal) oscillator.
 *
 * This function will block until the HSI is ready.
 */
void RCC_vEnableHSI()
{
	SET_BIT(RCC->CR, HSION);
	while (GET_BIT(RCC->CR, HSIRDY) == 0);
}

/**
 * @brief Disables the HSI (High-Speed Internal) oscillator.
 */
void RCC_vDisableHSI()
{
	CLR_BIT(RCC->CR, HSION);
}

/**
 * @brief Enables the LSE (Low-Speed External) oscillator.
 *
 * This function will block until the LSE is ready.
 */
void RCC_vEnableLSE()
{
	SET_BIT(RCC->BDCR, LSEON);
	while (GET_BIT(RCC->BDCR, LSERDY) == 0);
}

/**
 * @brief Disables the LSE (Low-Speed External) oscillator.
 */
void RCC_vDisableLSE()
{
	CLR_BIT(RCC->BDCR, LSEON);
}

/**
 * @brief Enables the LSI (Low-Speed Internal) oscillator.
 *
 * This function will block until the LSI is ready.
 */
void RCC_vEnableLSI()
{
	SET_BIT(RCC->CSR, LSION);
	while (GET_BIT(RCC->CSR, LSIRDY) == 0);
}

/**
 * @brief Disables the LSI (Low-Speed Internal) oscillator.
 */
void RCC_vDisableLSI()
{
	CLR_BIT(RCC->CSR, LSION);
}

/**
 * @brief Sets the APB1 (Advanced Peripheral Bus 1) prescaler.
 *
 * @param Copy_enPrescaler The prescaler value to be set (e.g., divide by 1, 2, 4, etc.)
 *
 * @return uint8 Returns OK (0) if the prescaler is set successfully, NOK (1) otherwise.
 */
uint8 RCC_u8SetAPB1Prescaler(APB_Prescaler Copy_enPrescaler)
{
	uint8 Local_u8ErrState = OK;

	if ((Copy_enPrescaler >= APB_PRESCALER_1) && (Copy_enPrescaler <= APB_PRESCALER_16))
	{
		RCC->CFGR = ((RCC->CFGR) & APB1_MASK) | (Copy_enPrescaler << APB1_POS);
	}
	else
	{
		Local_u8ErrState = NOK; // Invalid prescaler value
	}

	return Local_u8ErrState;
}

/**
 * @brief Sets the APB2 (Advanced Peripheral Bus 2) prescaler.
 *
 * @param Copy_enPrescaler The prescaler value to be set (e.g., divide by 1, 2, 4, etc.)
 *
 * @return uint8 Returns OK (0) if the prescaler is set successfully, NOK (1) otherwise.
 */
uint8 RCC_u8SetAPB2Prescaler(APB_Prescaler Copy_enPrescaler)
{
	uint8 Local_u8ErrState = OK;

	if ((Copy_enPrescaler >= APB_PRESCALER_1) && (Copy_enPrescaler <= APB_PRESCALER_16))
	{
		RCC->CFGR = ((RCC->CFGR) & APB2_MASK) | (Copy_enPrescaler << APB2_POS);
	}
	else
	{
		Local_u8ErrState = NOK; // Invalid prescaler value
	}

	return Local_u8ErrState;
}

/**
 * @brief Sets the AHB (Advanced High-Performance Bus) prescaler.
 *
 * @param Copy_enPrescaler The prescaler value to be set (e.g., divide by 1, 2, 4, etc.)
 *
 * @return uint8 Returns OK (0) if the prescaler is set successfully, NOK (1) otherwise.
 */
uint8 RCC_u8SetAHBPrescaler(AHB_Prescaler Copy_enPrescaler)
{
	uint8 Local_u8ErrState = OK;

	if ((Copy_enPrescaler >= AHB_PRESCALER_1) && (Copy_enPrescaler <= AHB_PRESCALER_512))
	{
		RCC->CFGR = ((RCC->CFGR) & AHB_MASK) | (Copy_enPrescaler << AHB_POS);
	}
	else
	{
		Local_u8ErrState = NOK; // Invalid prescaler value
	}

	return Local_u8ErrState;
}

/**
 * @brief Enables the clock for the specified peripheral.
 *
 * @param Copy_enPeriphral The peripheral to enable (e.g., GPIO, TIM, UART, etc.)
 *
 * This function determines which register (APB1, APB2, or AHB) to write to based on the peripheral.
 */
void RCC_vEnablePeriphralClock(PeripheralEnable Copy_enPeriphral)
{
	if ((Copy_enPeriphral & (1 << IsAPB1)) != 0)
	{
		CLR_BIT(Copy_enPeriphral, IsAPB1);
		RCC->APB1ENR |= Copy_enPeriphral;
	}
	else if ((Copy_enPeriphral & (1 << IsAPB2)) != 0)
	{
		CLR_BIT(Copy_enPeriphral, IsAPB2);
		RCC->APB2ENR |= Copy_enPeriphral;
	}
	else
	{
		RCC->AHBENR |= Copy_enPeriphral;
	}
}
