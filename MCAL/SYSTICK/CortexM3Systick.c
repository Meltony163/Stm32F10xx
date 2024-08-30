/*
 * CortexM3Systick.c
 *
 *  Created on: Aug 30, 2024
 *      Author: moame
 */
#include"CortexM3Systick.h"
#include"STD_TYPES.h"
#include"ErrType.h"
#include"BIT_MATH.h"

uint8 Glob_enuCallType;
void(* Glob_vFunctionPtr)();

// Function to disable the SysTick timer
void Systick_vDisable()
{
    CLR_BIT(SYSTICK->CSR, CSR_ENABLE);  // Clear the ENABLE bit to stop the SysTick timer
}

// Function to perform a blocking wait using the SysTick timer
uint8 Systick_u8WaitBlocking(uint32 Copy_u32TicksNumber, Clock Copy_enuClockSource)
{
    uint8 Local_u8ErrState = OK;  // Error state initialized to OK

    // Apply mask to ensure ticks are within valid range
    Copy_u32TicksNumber = Copy_u32TicksNumber & NumberOfTicks_MASK;

    // Set the clock source based on the provided enum value
    if (Copy_enuClockSource == ProcessorlClock)
    {
        SET_BIT(SYSTICK->CSR, CSR_CLKSOURCE);  // Use processor clock
    }
    else if (Copy_enuClockSource == ExternalClock)
    {
        CLR_BIT(SYSTICK->CSR, CSR_CLKSOURCE);  // Use external clock
    }
    else
    {
        Local_u8ErrState = NOK;  // Invalid clock source, set error state
    }

    if (Local_u8ErrState == OK)
    {
        SYSTICK->RVR = Copy_u32TicksNumber;  // Set reload value
        SYSTICK->CVR = 0u;                   // Clear current value register
        CLR_BIT(SYSTICK->CSR, CSR_TICKINT);  // Disable interrupt
        CLR_BIT(SYSTICK->CSR, CSR_COUNTFLAG);// Clear count flag
        SET_BIT(SYSTICK->CSR, CSR_ENABLE);   // Enable SysTick timer

        // Wait until the count flag is set, indicating the timer has reached zero
        while (GET_BIT(SYSTICK->CSR, CSR_COUNTFLAG) == 0);

        // Disable SysTick after wait is complete
        CLR_BIT(SYSTICK->CSR, CSR_ENABLE);
    }

    return Local_u8ErrState;  // Return error state
}

// Function to configure and start the SysTick timer with interrupts
uint8 Systick_u8WaitInterval(SystickConfig* Ptr_stConfig)
{
    uint8 Local_u8ErrState = OK;  // Error state initialized to OK

    // Check for NULL pointers
    if ((Ptr_stConfig != NULL) && (Ptr_stConfig->FunctionPtr != NULL))
    {
        Glob_vFunctionPtr = Ptr_stConfig->FunctionPtr;  // Store callback function pointer

        // Apply mask to ensure ticks are within valid range
        Ptr_stConfig->NumberOfTicks = (Ptr_stConfig->NumberOfTicks) & NumberOfTicks_MASK;

        // Set the clock source based on the provided enum value
        if (Ptr_stConfig->ClockSource == ProcessorlClock)
        {
            SET_BIT(SYSTICK->CSR, CSR_CLKSOURCE);  // Use processor clock
        }
        else if (Ptr_stConfig->ClockSource == ExternalClock)
        {
            CLR_BIT(SYSTICK->CSR, CSR_CLKSOURCE);  // Use external clock
        }
        else
        {
            Local_u8ErrState = NOK;  // Invalid clock source, set error state
        }

        // Set the call type (Single or Periodic)
        if (Ptr_stConfig->CallType == SingleCall)
        {
            Glob_enuCallType = SingleCall;
        }
        else if (Ptr_stConfig->CallType == PeriodicCall)
        {
            Glob_enuCallType = PeriodicCall;
        }
        else
        {
            Local_u8ErrState = NOK;  // Invalid call type, set error state
        }

        if (Local_u8ErrState == OK)
        {
            SYSTICK->RVR = Ptr_stConfig->NumberOfTicks;  // Set reload value
            SYSTICK->CVR = 0u;                           // Clear current value register
            SET_BIT(SYSTICK->CSR, CSR_TICKINT);          // Enable interrupt
            SET_BIT(SYSTICK->CSR, CSR_ENABLE);           // Enable SysTick timer
        }
    }
    else
    {
        Local_u8ErrState = NULL_PTR;  // NULL pointer error
    }

    return Local_u8ErrState;  // Return error state
}

// Function to get the remaining ticks before the next SysTick interrupt
uint32 Systick_u32GetRemaningTicks()
{
    // Return the current value register masked to 24-bits
    return (uint32)((SYSTICK->CVR) & NumberOfTicks_MASK);
}

// SysTick interrupt handler
void SysTick_Handler(void)
{
    // If the call type is SingleCall, disable the SysTick timer after the first interrupt
    if (Glob_enuCallType == SingleCall)
    {
        CLR_BIT(SYSTICK->CSR, CSR_ENABLE);
    }

    // If a callback function is set, execute it
    if (Glob_vFunctionPtr != NULL)
    {
        Glob_vFunctionPtr();
    }
}


