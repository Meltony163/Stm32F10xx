/*
 * CortexM3Systick.h
 *
 *  Created on: Aug 30, 2024
 *      Author: moame
 */

#ifndef CORTEXM3SYSTICK_H_
#define CORTEXM3SYSTICK_H_

/********************************************Library Include Start********************************************/
#include"STD_TYPES.h"
#include"ErrType.h"
#include"BIT_MATH.h"

/********************************************Library Include End********************************************/
/************************************SysTick Register Declaration Start************************************/

#define SYSTICK_BASE        (0xE000E010)           // Base address of the SysTick registers
#define SYSTICK             ((SYSTICK_REG*)SYSTICK_BASE)  // Pointer to SysTick registers

typedef struct {
    volatile uint32 CSR;   // Control and Status Register
    volatile uint32 RVR;   // Reload Value Register
    volatile uint32 CVR;   // Current Value Register
    volatile uint32 CALIB; // Calibration Value Register
} SYSTICK_REG;

#define CSR_COUNTFLAG       16u     // Count flag bit position
#define CSR_CLKSOURCE       2u      // Clock source bit position
#define CSR_TICKINT         1u      // Tick interrupt enable bit position
#define CSR_ENABLE          0u      // SysTick enable bit position
/*************************************SysTick Register Declaration End*************************************/

/************************************SysTick DataType Declaration Start************************************/

// Enumeration for SysTick clock source
typedef enum {
    ProcessorlClock,    // Use the processor clock as the SysTick clock source
    ExternalClock,      // Use an external clock as the SysTick clock source
} Clock;

// Enumeration for SysTick call type
typedef enum {
    SingleCall,         // SysTick will stop after a single interrupt
    PeriodicCall,       // SysTick will continue generating interrupts periodically
} Call;

// Configuration structure for SysTick
typedef struct {
    Clock ClockSource;             // Clock source to be used for SysTick
    uint32 NumberOfTicks;          // Number of ticks before SysTick generates an interrupt
    Call CallType;                 // Single or periodic call type
    void(* FunctionPtr)();         // Function pointer to the callback function
} SystickConfig;

/*************************************SysTick DataType Declaration End*************************************/

/************************************SysTick Functions Declaration Start************************************/

// Function to disable the SysTick timer
void Systick_vDisable();

// Function to perform a blocking wait using the SysTick timer
uint8 Systick_u8WaitBlocking(uint32 Copy_u32TicksNumber, Clock Copy_enuClockSource);

// Function to configure and start the SysTick timer with interrupts
uint8 Systick_u8WaitInterval(SystickConfig* Ptr_stConfig);

// Function to get the remaining ticks before the next SysTick interrupt
uint32 Systick_u32GetRemaningTicks();

/*************************************SysTick Functions Declaration End*************************************/

/************************************SysTick Macros Declaration Start************************************/

#define NumberOfTicks_MASK          0xFFFFFF  // Mask for valid tick count (24-bit value)

/*************************************SysTick Macros Declaration End*************************************/

#endif /* CORTEXM3SYSTICK_H_ */
