/*
 * CortexM3Nvic.c
 *
 *  Created on: Aug 30, 2024
 *      Author: moame
 */
#include "STD_TYPES.h"
#include "ErrType.h"
#include"CortexM3Nvic.h"

/**
 * @brief Enables the specified interrupt.
 *
 * @param Copy_enuInterrupt Interrupt number to enable.
 * @return uint8 Error status: OK if successful, NOK if the interrupt number is out of bounds.
 */
uint8 Nvic_u8Enableinterrupt(IRQn_Type Copy_enuInterrupt)
{
    uint8 Local_u8ErrState = OK;

    // Check if the interrupt number is within valid range
    if(((uint32)Copy_enuInterrupt >= (uint32)FIRST_IRQN) && ((uint32)Copy_enuInterrupt <= (uint32)LAST_IRQN))
    {
        // Set the appropriate bit in the Interrupt Set-Enable Register (ISER)
        NVIC->ISER[(uint32)((uint32)Copy_enuInterrupt >> 5)] = (uint32)((uint32)Copy_enuInterrupt & 31);
    }
    else
    {
        Local_u8ErrState = NOK;  // Return an error if the interrupt number is out of bounds
    }

    return Local_u8ErrState;
}

/**
 * @brief Disables the specified interrupt.
 *
 * @param Copy_enuInterrupt Interrupt number to disable.
 * @return uint8 Error status: OK if successful, NOK if the interrupt number is out of bounds.
 */
uint8 Nvic_u8Disableinterrupt(IRQn_Type Copy_enuInterrupt)
{
    uint8 Local_u8ErrState = OK;

    // Check if the interrupt number is within valid range
    if(((uint32)Copy_enuInterrupt >= (uint32)FIRST_IRQN) && ((uint32)Copy_enuInterrupt <= (uint32)LAST_IRQN))
    {
        // Set the appropriate bit in the Interrupt Clear-Enable Register (ICER)
        NVIC->ICER[(uint32)((uint32)Copy_enuInterrupt >> 5)] = (uint32)((uint32)Copy_enuInterrupt & 31);
    }
    else
    {
        Local_u8ErrState = NOK;  // Return an error if the interrupt number is out of bounds
    }

    return Local_u8ErrState;
}

/**
 * @brief Sets the pending bit for the specified interrupt.
 *
 * @param Copy_enuInterrupt Interrupt number to set as pending.
 * @return uint8 Error status: OK if successful, NOK if the interrupt number is out of bounds.
 */
uint8 Nvic_u8SetPendinginterrupt(IRQn_Type Copy_enuInterrupt)
{
    uint8 Local_u8ErrState = OK;

    // Check if the interrupt number is within valid range
    if(((uint32)Copy_enuInterrupt >= (uint32)FIRST_IRQN) && ((uint32)Copy_enuInterrupt <= (uint32)LAST_IRQN))
    {
        // Set the appropriate bit in the Interrupt Set-Pending Register (ISPR)
        NVIC->ISPR[(uint32)((uint32)Copy_enuInterrupt >> 5)] = (uint32)((uint32)Copy_enuInterrupt & 31);
    }
    else
    {
        Local_u8ErrState = NOK;  // Return an error if the interrupt number is out of bounds
    }

    return Local_u8ErrState;
}

/**
 * @brief Clears the pending bit for the specified interrupt.
 *
 * @param Copy_enuInterrupt Interrupt number to clear from pending.
 * @return uint8 Error status: OK if successful, NOK if the interrupt number is out of bounds.
 */
uint8 Nvic_u8ClearPendinginterrupt(IRQn_Type Copy_enuInterrupt)
{
    uint8 Local_u8ErrState = OK;

    // Check if the interrupt number is within valid range
    if(((uint32)Copy_enuInterrupt >= (uint32)FIRST_IRQN) && ((uint32)Copy_enuInterrupt <= (uint32)LAST_IRQN))
    {
        // Set the appropriate bit in the Interrupt Clear-Pending Register (ICPR)
        NVIC->ICPR[(uint32)((uint32)Copy_enuInterrupt >> 5)] = (uint32)((uint32)Copy_enuInterrupt & 31);
    }
    else
    {
        Local_u8ErrState = NOK;  // Return an error if the interrupt number is out of bounds
    }

    return Local_u8ErrState;
}

/**
 * @brief Gets the active state of the specified interrupt.
 *
 * @param Copy_enuInterrupt Interrupt number to check for active state.
 * @return uint8 Active state: Active, NOT_ACTIVE, or NOK if the interrupt number is out of bounds.
 */
uint8 Nvic_u8GetActiveinterrupt(IRQn_Type Copy_enuInterrupt)
{
    uint8 Local_u8ReturnState;

    // Check if the interrupt number is within valid range
    if(((uint32)Copy_enuInterrupt >= (uint32)FIRST_IRQN) && ((uint32)Copy_enuInterrupt <= (uint32)LAST_IRQN))
    {
        // Return Active or NOT_ACTIVE depending on the status of the interrupt
        Local_u8ReturnState = ((NVIC->IABR[(uint32)((uint32)Copy_enuInterrupt >> 5)]) & (1 << ((uint32)((uint32)Copy_enuInterrupt & 31)))) != 0 ? Active : NOT_ACTIVE;
    }
    else
    {
        Local_u8ReturnState = NOK;  // Return an error if the interrupt number is out of bounds
    }

    return Local_u8ReturnState;
}

/**
 * @brief Sets the priority of the specified interrupt.
 *
 * @param Copy_enuInterrupt Interrupt number to set the priority for.
 * @param Copy_u8InterruptPriority Priority level to assign to the interrupt.
 * @return uint8 Error status: OK if successful, NOK if the interrupt number is out of bounds.
 */
uint8 Nvic_u8SetinterruptPriority(IRQn_Type Copy_enuInterrupt, uint8 Copy_u8InterruptPriority)
{
    uint8 Local_u8ErrState = OK;

    // Check if the interrupt number is within valid range
    if(((uint32)Copy_enuInterrupt >= (uint32)FIRST_IRQN) && ((uint32)Copy_enuInterrupt <= (uint32)LAST_IRQN))
    {
        // Set the priority in the Interrupt Priority Register (IP) after shifting to the correct position
        NVIC->IP[(uint32)Copy_enuInterrupt] = (uint8)(Copy_u8InterruptPriority << NVIC_PRIORITY_POS);
    }
    else
    {
        Local_u8ErrState = NOK;  // Return an error if the interrupt number is out of bounds
    }

    return Local_u8ErrState;
}

/**
 * @brief Configures the priority grouping for the NVIC.
 *
 * @param Copy_enuPriorityGroup Priority grouping configuration.
 * @return uint8 Error status: OK if successful, NOK if the priority group is out of bounds.
 */
uint8 NVIC_u8SetPriorityGroup(IRQ_PriorityGroup Copy_enuPriorityGroup)
{
    uint8 Local_u8ErrState = OK;

    // Check if the priority group is within valid range
    if((Copy_enuPriorityGroup <= PriorityGroup5) && (Copy_enuPriorityGroup >= PriorityGroup1))
    {
        // Set the priority grouping in the Application Interrupt and Reset Control Register (AIRCR)
        SCB_AIRCR = (uint32)((AIRCR_MASK & SCB_AIRCR) | (VECTKEYSTAT << VECTKEYSTAT_POS) | (Copy_enuPriorityGroup << SCB_PRIORITY_POS));
    }
    else
    {
        Local_u8ErrState = NOK;  // Return an error if the priority group is out of bounds
    }

    return Local_u8ErrState;
}

