/*
 * FreeRTOS Kernel V10.0.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. If you wish to use our Amazon
 * FreeRTOS name, please do so in a fair use way that does not cause confusion.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Xilinx includes. */
#include "xscugic.h"

#ifndef configINTERRUPT_CONTROLLER_BASE_ADDRESS
	#error configINTERRUPT_CONTROLLER_BASE_ADDRESS must be defined.  Refer to Cortex-A equivalent: http://www.freertos.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#ifndef configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET
	#error configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET must be defined.  Refer to Cortex-A equivalent: http://www.freertos.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#ifndef configUNIQUE_INTERRUPT_PRIORITIES
	#error configUNIQUE_INTERRUPT_PRIORITIES must be defined.  Refer to Cortex-A equivalent: http://www.freertos.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#ifndef configSETUP_TICK_INTERRUPT
	#error configSETUP_TICK_INTERRUPT() must be defined.  Refer to Cortex-A equivalent: http://www.freertos.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif /* configSETUP_TICK_INTERRUPT */

#ifndef configMAX_API_CALL_INTERRUPT_PRIORITY
	#error configMAX_API_CALL_INTERRUPT_PRIORITY must be defined.  Refer to Cortex-A equivalent: http://www.freertos.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#if configMAX_API_CALL_INTERRUPT_PRIORITY == 0
	#error configMAX_API_CALL_INTERRUPT_PRIORITY must not be set to 0
#endif

#if configMAX_API_CALL_INTERRUPT_PRIORITY > configUNIQUE_INTERRUPT_PRIORITIES
	#error configMAX_API_CALL_INTERRUPT_PRIORITY must be less than or equal to configUNIQUE_INTERRUPT_PRIORITIES as the lower the numeric priority value the higher the logical interrupt priority
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1
	/* Check the configuration. */
	#if( configMAX_PRIORITIES > 32 )
		#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
	#endif
#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/* In case security extensions are implemented. */
#if configMAX_API_CALL_INTERRUPT_PRIORITY <= ( configUNIQUE_INTERRUPT_PRIORITIES / 2 )
	#error configMAX_API_CALL_INTERRUPT_PRIORITY must be greater than ( configUNIQUE_INTERRUPT_PRIORITIES / 2 )
#endif

/* Some vendor specific files default configCLEAR_TICK_INTERRUPT() in
portmacro.h. */
#ifndef configCLEAR_TICK_INTERRUPT
	#define configCLEAR_TICK_INTERRUPT()
#endif

/* A critical section is exited when the critical section nesting count reaches
this value. */
#define portNO_CRITICAL_NESTING			( ( uint32_t ) 0 )

/* In all GICs 255 can be written to the priority mask register to unmask all
(but the lowest) interrupt priority. */
#define portUNMASK_VALUE				( 0xFFUL )

/* Tasks are not created with a floating point context, but can be given a
floating point context after they have been created.  A variable is stored as
part of the tasks context that holds portNO_FLOATING_POINT_CONTEXT if the task
does not have an FPU context, or any other value if the task does have an FPU
context. */
#define portNO_FLOATING_POINT_CONTEXT	( ( StackType_t ) 0 )

/* Constants required to setup the initial task context. */
#define portINITIAL_SPSR				( ( StackType_t ) 0x1f ) /* System mode, ARM mode, IRQ enabled FIQ enabled. */
#define portTHUMB_MODE_BIT				( ( StackType_t ) 0x20 )
#define portINTERRUPT_ENABLE_BIT		( 0x80UL )
#define portTHUMB_MODE_ADDRESS			( 0x01UL )

/* Used by portASSERT_IF_INTERRUPT_PRIORITY_INVALID() when ensuring the binary
point is zero. */
#define portBINARY_POINT_BITS			( ( uint8_t ) 0x03 )

/* Masks all bits in the APSR other than the mode bits. */
#define portAPSR_MODE_BITS_MASK			( 0x1F )

/* The value of the mode bits in the APSR when the CPU is executing in user
mode. */
#define portAPSR_USER_MODE				( 0x10 )

/* The critical section macros only mask interrupts up to an application
determined priority level.  Sometimes it is necessary to turn interrupt off in
the CPU itself before modifying certain hardware registers. */
#define portCPU_IRQ_DISABLE()										\
	__asm volatile ( "CPSID i" ::: "memory" );						\
	__asm volatile ( "DSB" );										\
	__asm volatile ( "ISB" );

#define portCPU_IRQ_ENABLE()										\
	__asm volatile ( "CPSIE i" ::: "memory" );						\
	__asm volatile ( "DSB" );										\
	__asm volatile ( "ISB" );


/* Macro to unmask all interrupt priorities. */
#define portCLEAR_INTERRUPT_MASK()									\
{																	\
	portCPU_IRQ_DISABLE();											\
	portICCPMR_PRIORITY_MASK_REGISTER = portUNMASK_VALUE;			\
	__asm volatile (	"DSB		\n"								\
						"ISB		\n" );							\
	portCPU_IRQ_ENABLE();											\
}

#define portINTERRUPT_PRIORITY_REGISTER_OFFSET		0x400UL
#define portMAX_8_BIT_VALUE							( ( uint8_t ) 0xff )
#define portBIT_0_SET								( ( uint8_t ) 0x01 )

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case is messes up unwinding of the stack in the
debugger. */
#ifdef configTASK_RETURN_ADDRESS
	#define portTASK_RETURN_ADDRESS	configTASK_RETURN_ADDRESS
#else
	#define portTASK_RETURN_ADDRESS	prvTaskExitError
#endif

/*-----------------------------------------------------------*/

/*
 * Starts the first task executing.  This function is necessarily written in
 * assembly code so is implemented in portASM.s.
 */
extern void vPortRestoreTaskContext( void );

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/* A variable is used to keep track of the critical section nesting.  This
variable has to be stored as part of the task context and must be initialised to
a non zero value to ensure interrupts don't inadvertently become unmasked before
the scheduler starts.  As it is stored as part of the task context it will
automatically be set to 0 when the first task is started. */
volatile uint32_t ulCriticalNesting = 9999UL;

/* 
 * The instance of the interrupt controller used by this port.  This is required
 * by the Xilinx library API functions.
 */
extern XScuGic xInterruptController;

/* Saved as part of the task context.  If ulPortTaskHasFPUContext is non-zero then
a floating point context must be saved and restored for the task. */
uint32_t ulPortTaskHasFPUContext = pdFALSE;

/* Set to 1 to pend a context switch from an ISR. */
uint32_t ulPortYieldRequired = pdFALSE;

/* Counts the interrupt nesting depth.  A context switch is only performed if
if the nesting depth is 0. */
uint32_t ulPortInterruptNesting = 0UL;

/* Used in asm code. */
__attribute__(( used )) const uint32_t ulICCIAR = portICCIAR_INTERRUPT_ACKNOWLEDGE_REGISTER_ADDRESS;
__attribute__(( used )) const uint32_t ulICCEOIR = portICCEOIR_END_OF_INTERRUPT_REGISTER_ADDRESS;
__attribute__(( used )) const uint32_t ulICCPMR	= portICCPMR_PRIORITY_MASK_REGISTER_ADDRESS;
__attribute__(( used )) const uint32_t ulMaxAPIPriorityMask = ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );

/*-----------------------------------------------------------*/
/*
 * Initialise the interrupt controller instance.
 */
static int32_t prvInitialiseInterruptController( void );

/* Ensure the interrupt controller instance variable is initialised before it is
 * used, and that the initialisation only happens once.
 */
static int32_t prvEnsureInterruptControllerIsInitialised( void );

/*
 * See header file for description.
 */
#if( portUSING_MPU_WRAPPERS == 1 )
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters, BaseType_t xRunPrivileged )
#else
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
#endif
{
	/* Setup the initial stack of the task.  The stack is set exactly as
	expected by the portRESTORE_CONTEXT() macro.

	The fist real value on the stack is the status register, which is set for
	system mode, with interrupts enabled.  A few NULLs are added first to ensure
	GDB does not try decoding a non-existent return address. */
	*pxTopOfStack = ( StackType_t ) NULL;
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) NULL;
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) NULL;
	pxTopOfStack--;

#if( portUSING_MPU_WRAPPERS == 1 )

	if( xRunPrivileged == pdTRUE )
	{
		*pxTopOfStack = portINITIAL_SPSR_PRIVILEGED;
	}
	else
	{
		*pxTopOfStack = portINITIAL_SPSR_UNPRIVILEGED;
	}
#else
	*pxTopOfStack = ( StackType_t ) portINITIAL_SPSR;
#endif

	if( ( ( uint32_t ) pxCode & portTHUMB_MODE_ADDRESS ) != 0x00UL )
	{
		/* The task will start in THUMB mode. */
		*pxTopOfStack |= portTHUMB_MODE_BIT;
	}

	pxTopOfStack--;

	/* Next the return address, which in this case is the start of the task. */
	*pxTopOfStack = ( StackType_t ) pxCode;
	pxTopOfStack--;

	/* Next all the registers other than the stack pointer. */
	*pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;	/* R14 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x12121212;	/* R12 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x11111111;	/* R11 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x10101010;	/* R10 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x09090909;	/* R9 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x08080808;	/* R8 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x07070707;	/* R7 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x06060606;	/* R6 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x05050505;	/* R5 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x04040404;	/* R4 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x03030303;	/* R3 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x02020202;	/* R2 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x01010101;	/* R1 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) pvParameters; /* R0 */
	pxTopOfStack--;

	/* The task will start with a critical nesting count of 0 as interrupts are
	enabled. */
	*pxTopOfStack = portNO_CRITICAL_NESTING;
	pxTopOfStack--;

	/* The task will start without a floating point context.  A task that uses
	the floating point hardware must call vPortTaskUsesFPU() before executing
	any floating point instructions. */
	*pxTopOfStack = portNO_FLOATING_POINT_CONTEXT;


	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
	/* A function that implements a task must not exit or attempt to return to
	its caller as there is nothing to return to.  If a task wants to exit it
	should instead call vTaskDelete( NULL ).

	Artificially force an assert() to be triggered if configASSERT() is
	defined, then stop here so application writers can catch the error. */
	configASSERT( ulPortInterruptNesting == ~0UL );
	portDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

BaseType_t xPortInstallInterruptHandler( uint8_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef )
{
int32_t lReturn;

	/* An API function is provided to install an interrupt handler because the
	interrupt controller instance variable is private to this file. */
	lReturn = prvEnsureInterruptControllerIsInitialised();
	if( lReturn == pdPASS )
	{
		lReturn = XScuGic_Connect( &xInterruptController, ucInterruptID, pxHandler, pvCallBackRef );
	}
	if( lReturn == XST_SUCCESS )
	{
		lReturn = pdPASS;
	}
	configASSERT( lReturn == pdPASS );

	return lReturn;
}
/*-----------------------------------------------------------*/

static int32_t prvEnsureInterruptControllerIsInitialised( void )
{
static int32_t lInterruptControllerInitialised = pdFALSE;
int32_t lReturn;

	/* Ensure the interrupt controller instance variable is initialised before
	it is used, and that the initialisation only happens once. */
	if( lInterruptControllerInitialised != pdTRUE )
	{
		lReturn = prvInitialiseInterruptController();

		if( lReturn == pdPASS )
		{
			lInterruptControllerInitialised = pdTRUE;
		}
	}
	else
	{
		lReturn = pdPASS;
	}

	return lReturn;
}
/*-----------------------------------------------------------*/

static int32_t prvInitialiseInterruptController( void )
{
BaseType_t xStatus;
XScuGic_Config *pxGICConfig;

	/* Initialize the interrupt controller driver. */
	pxGICConfig = XScuGic_LookupConfig( configINTERRUPT_CONTROLLER_DEVICE_ID );
	xStatus = XScuGic_CfgInitialize( &xInterruptController, pxGICConfig, pxGICConfig->CpuBaseAddress );
	/* Connect the interrupt controller interrupt handler to the hardware
	interrupt handling logic in the ARM processor. */
	Xil_ExceptionRegisterHandler( XIL_EXCEPTION_ID_IRQ_INT,
								( Xil_ExceptionHandler ) XScuGic_InterruptHandler,
								&xInterruptController);
	/* Enable interrupts in the ARM. */
	Xil_ExceptionEnable();
	
		if( xStatus == XST_SUCCESS )
		{
			xStatus = pdPASS;
		}
		else
		{
			xStatus = pdFAIL;
		}	
	configASSERT( xStatus == pdPASS );

	return xStatus;
}
/*-----------------------------------------------------------*/

void vPortEnableInterrupt( uint8_t ucInterruptID )
{
int32_t lReturn;

	/* An API function is provided to enable an interrupt in the interrupt
	controller. */
	lReturn = prvEnsureInterruptControllerIsInitialised();
	if( lReturn == pdPASS )
	{
		XScuGic_Enable( &xInterruptController, ucInterruptID );
	}	
	configASSERT( lReturn );
}
/*-----------------------------------------------------------*/

void vPortDisableInterrupt( uint8_t ucInterruptID )
{
int32_t lReturn;

	/* An API function is provided to disable an interrupt in the interrupt
	controller. */
	lReturn = prvEnsureInterruptControllerIsInitialised();
	if( lReturn == pdPASS )
	{
		XScuGic_Disable( &xInterruptController, ucInterruptID );
	}
	configASSERT( lReturn );
}
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
uint32_t ulAPSR, ulCycles = 8; /* 8 bits per byte. */

	#if( configASSERT_DEFINED == 1 )
	{
		volatile uint32_t ulOriginalPriority;
		volatile uint8_t * const pucFirstUserPriorityRegister = ( volatile uint8_t * const ) ( configINTERRUPT_CONTROLLER_BASE_ADDRESS + portINTERRUPT_PRIORITY_REGISTER_OFFSET );
		volatile uint8_t ucMaxPriorityValue;

		/* Determine how many priority bits are implemented in the GIC.

		Save the interrupt priority value that is about to be clobbered. */
		ulOriginalPriority = *pucFirstUserPriorityRegister;

		/* Determine the number of priority bits available.  First write to
		all possible bits. */
		*pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

		/* Read the value back to see how many bits stuck. */
		ucMaxPriorityValue = *pucFirstUserPriorityRegister;

		/* Shift to the least significant bits. */
		while( ( ucMaxPriorityValue & portBIT_0_SET ) != portBIT_0_SET )
		{
			ucMaxPriorityValue >>= ( uint8_t ) 0x01;

			/* If ulCycles reaches 0 then ucMaxPriorityValue must have been
			read as 0, indicating a misconfiguration. */
			ulCycles--;
			if( ulCycles == 0 )
			{
				break;
			}
		}

		/* Sanity check configUNIQUE_INTERRUPT_PRIORITIES matches the read
		value. */
//		configASSERT( ucMaxPriorityValue == portLOWEST_INTERRUPT_PRIORITY );

		/* Restore the clobbered interrupt priority register to its original
		value. */
		*pucFirstUserPriorityRegister = ulOriginalPriority;
	}
	#endif /* conifgASSERT_DEFINED */

	/* Only continue if the CPU is not in User mode.  The CPU must be in a
	Privileged mode for the scheduler to start. */
	__asm volatile ( "MRS %0, APSR" : "=r" ( ulAPSR ) :: "memory" );
	ulAPSR &= portAPSR_MODE_BITS_MASK;
	configASSERT( ulAPSR != portAPSR_USER_MODE );

	if( ulAPSR != portAPSR_USER_MODE )
	{
		/* Only continue if the binary point value is set to its lowest possible
		setting.  See the comments in vPortValidateInterruptPriority() below for
		more information. */
		configASSERT( ( portICCBPR_BINARY_POINT_REGISTER & portBINARY_POINT_BITS ) <= portMAX_BINARY_POINT_VALUE );

		if( ( portICCBPR_BINARY_POINT_REGISTER & portBINARY_POINT_BITS ) <= portMAX_BINARY_POINT_VALUE )
		{
			/* Interrupts are turned off in the CPU itself to ensure tick does
			not execute	while the scheduler is being started.  Interrupts are
			automatically turned back on in the CPU when the first task starts
			executing. */
			portCPU_IRQ_DISABLE();

			void setupMPU(void);

            setupMPU();

			/* Start the timer that generates the tick ISR. */
			configSETUP_TICK_INTERRUPT();

			/* Start the first task executing. */
			vPortRestoreTaskContext();
		}
	}

	/* Will only get here if vTaskStartScheduler() was called with the CPU in
	a non-privileged mode or the binary point register was not set to its lowest
	possible value.  prvTaskExitError() is referenced to prevent a compiler
	warning about it being defined but not referenced in the case that the user
	defines their own exit address. */
	( void ) prvTaskExitError;
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* Not implemented in ports where there is nothing to return to.
	Artificially force an assert. */
	configASSERT( ulCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
    /* CPU can't modify A,I of F bit of cpsr if in user mode */
#if( portUSING_MPU_WRAPPERS == 1 )
	BaseType_t xRunningPrivileged = xPortRaisePrivilege();
#endif

	/* Mask interrupts up to the max syscall interrupt priority. */
	ulPortSetInterruptMask();

	/* Now interrupts are disabled ulCriticalNesting can be accessed
	directly.  Increment ulCriticalNesting to keep a count of how many times
	portENTER_CRITICAL() has been called. */
	ulCriticalNesting++;

	/* This is not the interrupt safe version of the enter critical function so
	assert() if it is being called from an interrupt context.  Only API
	functions that end in "FromISR" can be used in an interrupt.  Only assert if
	the critical nesting count is 1 to protect against recursive calls if the
	assert function also uses a critical section. */
	if( ulCriticalNesting == 1 )
	{
		configASSERT( ulPortInterruptNesting == 0 );
	}

   /* If we were in usre mode, return to it. */
#if( portUSING_MPU_WRAPPERS == 1 )
    vPortResetPrivilege( xRunningPrivileged );
#endif

}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
    /* CPU can't modify A,I of F bit of cpsr if in user mode */
#if( portUSING_MPU_WRAPPERS == 1 )
	BaseType_t xRunningPrivileged = xPortRaisePrivilege();
#endif

	if( ulCriticalNesting > portNO_CRITICAL_NESTING )
	{
		/* Decrement the nesting count as the critical section is being
		exited. */
		ulCriticalNesting--;

		/* If the nesting level has reached zero then all interrupt
		priorities must be re-enabled. */
		if( ulCriticalNesting == portNO_CRITICAL_NESTING )
		{
			/* Critical nesting has reached zero so all interrupt priorities
			should be unmasked. */
			portCLEAR_INTERRUPT_MASK();
		}
	}

   /* If we were in usre mode, return to it. */
#if( portUSING_MPU_WRAPPERS == 1 )
    vPortResetPrivilege( xRunningPrivileged );
#endif
}
/*-----------------------------------------------------------*/

void FreeRTOS_Tick_Handler( void )
{
	/* Set interrupt mask before altering scheduler structures.   The tick
	handler runs at the lowest priority, so interrupts cannot already be masked,
	so there is no need to save and restore the current mask value.  It is
	necessary to turn off interrupts in the CPU itself while the ICCPMR is being
	updated. */
	portCPU_IRQ_DISABLE();
	portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
	__asm volatile (	"dsb		\n"
						"isb		\n" ::: "memory" );
	portCPU_IRQ_ENABLE();

	/* Increment the RTOS tick. */
	if( xTaskIncrementTick() != pdFALSE )
	{
		ulPortYieldRequired = pdTRUE;
	}

	/* Ensure all interrupt priorities are active again. */
	portCLEAR_INTERRUPT_MASK();
	configCLEAR_TICK_INTERRUPT();
}
/*-----------------------------------------------------------*/

void vPortTaskUsesFPU( void )
{
uint32_t ulInitialFPSCR = 0;

	/* A task is registering the fact that it needs an FPU context.  Set the
	FPU flag (which is saved as part of the task context). */
	ulPortTaskHasFPUContext = pdTRUE;

	/* Initialise the floating point status register. */
	__asm volatile ( "FMXR 	FPSCR, %0" :: "r" (ulInitialFPSCR) : "memory" );
}
/*-----------------------------------------------------------*/

void vPortClearInterruptMask( uint32_t ulNewMaskValue )
{
	if( ulNewMaskValue == pdFALSE )
	{
		portCLEAR_INTERRUPT_MASK();
	}
}
/*-----------------------------------------------------------*/

uint32_t ulPortSetInterruptMask( void )
{
uint32_t ulReturn;

	/* Interrupt in the CPU must be turned off while the ICCPMR is being
	updated. */
	portCPU_IRQ_DISABLE();
	if( portICCPMR_PRIORITY_MASK_REGISTER == ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT ) )
	{
		/* Interrupts were already masked. */
		ulReturn = pdTRUE;
	}
	else
	{
		ulReturn = pdFALSE;
		portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
		__asm volatile (	"dsb		\n"
							"isb		\n" ::: "memory" );
	}
	portCPU_IRQ_ENABLE();

	return ulReturn;
}
/*-----------------------------------------------------------*/

#if( configASSERT_DEFINED == 1 )

	void vPortValidateInterruptPriority( void )
	{
		/* The following assertion will fail if a service routine (ISR) for
		an interrupt that has been assigned a priority above
		configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
		function.  ISR safe FreeRTOS API functions must *only* be called
		from interrupts that have been assigned a priority at or below
		configMAX_SYSCALL_INTERRUPT_PRIORITY.

		Numerically low interrupt priority numbers represent logically high
		interrupt priorities, therefore the priority of the interrupt must
		be set to a value equal to or numerically *higher* than
		configMAX_SYSCALL_INTERRUPT_PRIORITY.

		FreeRTOS maintains separate thread and ISR API functions to ensure
		interrupt entry is as fast and simple as possible. */

		configASSERT( portICCRPR_RUNNING_PRIORITY_REGISTER >= ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT ) );

		/* Priority grouping:  The interrupt controller (GIC) allows the bits
		that define each interrupt's priority to be split between bits that
		define the interrupt's pre-emption priority bits and bits that define
		the interrupt's sub-priority.  For simplicity all bits must be defined
		to be pre-emption priority bits.  The following assertion will fail if
		this is not the case (if some bits represent a sub-priority).

		The priority grouping is configured by the GIC's binary point register
		(ICCBPR).  Writing 0 to ICCBPR will ensure it is set to its lowest
		possible value (which may be above 0). */
		configASSERT( ( portICCBPR_BINARY_POINT_REGISTER & portBINARY_POINT_BITS ) <= portMAX_BINARY_POINT_VALUE );
	}

#endif /* configASSERT_DEFINED */

static const struct {
	u64 size;
	unsigned int encoding;
}region_size[] = {
	{ 0x20, REGION_32B },
	{ 0x40, REGION_64B },
	{ 0x80, REGION_128B },
	{ 0x100, REGION_256B },
	{ 0x200, REGION_512B },
	{ 0x400, REGION_1K },
	{ 0x800, REGION_2K },
	{ 0x1000, REGION_4K },
	{ 0x2000, REGION_8K },
	{ 0x4000, REGION_16K },
	{ 0x8000, REGION_32K },
	{ 0x10000, REGION_64K },
	{ 0x20000, REGION_128K },
	{ 0x40000, REGION_256K },
	{ 0x80000, REGION_512K },
	{ 0x100000, REGION_1M },
	{ 0x200000, REGION_2M },
	{ 0x400000, REGION_4M },
	{ 0x800000, REGION_8M },
	{ 0x1000000, REGION_16M },
	{ 0x2000000, REGION_32M },
	{ 0x4000000, REGION_64M },
	{ 0x8000000, REGION_128M },
	{ 0x10000000, REGION_256M },
	{ 0x20000000, REGION_512M },
	{ 0x40000000, REGION_1G },
	{ 0x80000000, REGION_2G },
	{ 0x100000000, REGION_4G },
};

uint32_t getSizeEncoding(uint32_t size)
{
    uint32_t Regionsize = 0;

	/* Lookup the size.  */
	for (size_t i = 0; i < sizeof region_size / sizeof region_size[0]; i++) {
		if (size <= region_size[i].size) {
			Regionsize = region_size[i].encoding;
			break;
		}
	}

    return Regionsize;
}

void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t ulStackDepth )
{
	/* Called at task creation */
	if(pxBottomOfStack != NULL)
	{

		/* Stack depth is in word, we want bytes  */
		ulStackDepth *= 4;

	    /* First region is stack, always fill it. */
		xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress = (uint32_t) pxBottomOfStack;

		xMPUSettings->xRegion[ 0 ].ulRegionSize = (getSizeEncoding(ulStackDepth) << 1) | REGION_EN;

		xMPUSettings->xRegion[ 0 ].ulRegionAttribute = PRIV_RW_USER_RW | NORM_NSHARED_WB_WA | EXECUTE_NEVER;
	
		/* Caller passed nothing, Invalidate all other regions. */
		if( xRegions == NULL )
		{
			
			for( size_t i = 1; i <= portNUM_CONFIGURABLE_REGIONS; i++ )
			{
			    xMPUSettings->xRegion[ i ].ulRegionBaseAddress = 0x00000000;
    		    xMPUSettings->xRegion[ i ].ulRegionSize = 0x00000000;
			    xMPUSettings->xRegion[ i ].ulRegionAttribute = 0UL;
			}
		}
		/* Caller passed something we have to parse */
		else
		{
			size_t lIndex = 0;
			
			for( size_t i = 1; i <= portNUM_CONFIGURABLE_REGIONS; i++ )
			{
				/* If the region is valid, parse it */
				if( ( xRegions[ lIndex ] ).ulLengthInBytes > 0UL )
				{
					xMPUSettings->xRegion[ i ].ulRegionBaseAddress = ( uint32_t ) xRegions[ lIndex ].pvBaseAddress;
                	xMPUSettings->xRegion[ i ].ulRegionSize        = (getSizeEncoding(( uint32_t ) xRegions[ lIndex ].ulLengthInBytes) << 1 )| REGION_EN;
					xMPUSettings->xRegion[ i ].ulRegionAttribute   = xRegions[lIndex].ulParameters;
				}
				/* Otherwise, invalidate the region */
				else
				{
					/* Invalidate the region. */
			        xMPUSettings->xRegion[ i ].ulRegionBaseAddress = 0x00000000;
    		        xMPUSettings->xRegion[ i ].ulRegionSize        = 0x00000000;
			        xMPUSettings->xRegion[ i ].ulRegionAttribute   = 0UL;
				}

                lIndex++;
			}
		}
	}

	/* Stack pointer is null, the OS wants to update mpu regions of a task */
	else
	{
		size_t lIndex = 0;

		

		vPortEnterCritical();

		{
			/* User defined regions start at 13 */
			size_t mpuRegionNumber = 13;
			
			extern StaticTask_t * volatile pxCurrentTCB;

			/* Big hack to check if the running task is the one requested to modify its MPU regions, based on the pointer received  */
			size_t isCurrentTask = ((&pxCurrentTCB->xDummy2) == xMPUSettings) ? 1 : 0;

			for( size_t i = 1; i <= portNUM_CONFIGURABLE_REGIONS; i++ )
			{
				/* The region is valid, parse it */
				if( ( xRegions[ lIndex ] ).ulLengthInBytes > 0UL )
				{
					xMPUSettings->xRegion[ i ].ulRegionBaseAddress = ( uint32_t ) xRegions[ lIndex ].pvBaseAddress;
	                xMPUSettings->xRegion[ i ].ulRegionSize        = (getSizeEncoding(( uint32_t ) xRegions[ lIndex ].ulLengthInBytes) << 1 )| REGION_EN;
					xMPUSettings->xRegion[ i ].ulRegionAttribute   = xRegions[lIndex].ulParameters;
	
				}
				/* Invalidate it */
				else
				{
					/* Invalidate the region. */
			        xMPUSettings->xRegion[ i ].ulRegionBaseAddress = 0x00000000;
	    	        xMPUSettings->xRegion[ i ].ulRegionSize        = 0x00000000;
			        xMPUSettings->xRegion[ i ].ulRegionAttribute   = 0UL;
				}
	
				lIndex++;
	
				/* If it's the current task, update right now the MPU registers because the calling task might need the regions active right now */
				if(isCurrentTask)
				{
					dsb();
					mtcp(XREG_CP15_MPU_MEMORY_REG_NUMBER,mpuRegionNumber);
					isb();
					mtcp(XREG_CP15_MPU_REG_BASEADDR,xMPUSettings->xRegion[ i ].ulRegionBaseAddress);
					mtcp(XREG_CP15_MPU_REG_ACCESS_CTRL,xMPUSettings->xRegion[ i ].ulRegionAttribute);
					mtcp(XREG_CP15_MPU_REG_SIZE_EN,xMPUSettings->xRegion[ i ].ulRegionSize);
					dsb();
					isb();						/* synchronize context on this processor */

					mpuRegionNumber++;
				}
			}
		}

		vPortExitCritical();
	}

}

int32_t vPortTaskAllocateNextMPURegion( TaskHandle_t xTask, const MemoryRegion_t * const pxRegions )
{
	int32_t mpuRegionNumber = -1;

	BaseType_t xRunningPrivileged = xPortRaisePrivilege();

	vPortEnterCritical();
	{
		extern StaticTask_t * volatile pxCurrentTCB;

		/* Get the array of mpu settings*/
		xMPU_SETTINGS *xMPUSettings = (xTask == NULL) ? (&(pxCurrentTCB->xDummy2)) : &(((StaticTask_t *)xTask)->xDummy2);

		/* Is is request from the running task? */
		size_t isCurrentTask = (xTask == NULL) ? 1 : 0;

		/* We start at 1, because index 0 is the stack region, we don't want to mess with that */
		for( size_t i = 1; i <= portNUM_CONFIGURABLE_REGIONS; i++ )
		{
			if( (( xMPUSettings->xRegion[ i ] ).ulRegionBaseAddress == 0)  && (( xMPUSettings->xRegion[ i ] ).ulRegionSize == 0) && (( xMPUSettings->xRegion[ i ] ).ulRegionAttribute == 0))
			{
				/* We found a region. */
				xMPUSettings->xRegion[ i ].ulRegionBaseAddress = ( uint32_t ) pxRegions->pvBaseAddress;
                xMPUSettings->xRegion[ i ].ulRegionSize        = (getSizeEncoding(( uint32_t ) pxRegions->ulLengthInBytes) << 1 )| REGION_EN;
				xMPUSettings->xRegion[ i ].ulRegionAttribute   = pxRegions->ulParameters;

				mpuRegionNumber = i;

				/* If it's the current task, update right now the MPU registers because the calling task might need the regions active right now */
				if(isCurrentTask)
				{
					dsb();
					mtcp(XREG_CP15_MPU_MEMORY_REG_NUMBER, 12 + mpuRegionNumber);
					isb();
					mtcp(XREG_CP15_MPU_REG_BASEADDR,xMPUSettings->xRegion[ i ].ulRegionBaseAddress);
					mtcp(XREG_CP15_MPU_REG_ACCESS_CTRL,xMPUSettings->xRegion[ i ].ulRegionAttribute);
					mtcp(XREG_CP15_MPU_REG_SIZE_EN,xMPUSettings->xRegion[ i ].ulRegionSize);
					dsb();
					isb();						/* synchronize context on this processor */

				}
				
				break;;
			}	
		}
	
	}

	vPortExitCritical();

	vPortResetPrivilege( xRunningPrivileged );

	return mpuRegionNumber;
}


void vPortTaskClearMPURegion( TaskHandle_t xTask, int32_t regionNumber)
{
	configASSERT( regionNumber >= 1 && regionNumber <= 3);

	BaseType_t xRunningPrivileged = xPortRaisePrivilege();
	
	vPortEnterCritical();
	{
		extern StaticTask_t * volatile pxCurrentTCB;

		/* Get the array of mpu settings*/
		xMPU_SETTINGS *xMPUSettings = (xTask == NULL) ? (&(pxCurrentTCB->xDummy2)) : &(((StaticTask_t *)xTask)->xDummy2);

		/* Is is request from the running task? */
		size_t isCurrentTask = (xTask == NULL) ? 1 : 0;

		/* Invalidate the region. */
		xMPUSettings->xRegion[ regionNumber ].ulRegionBaseAddress = 0x00000000;
		xMPUSettings->xRegion[ regionNumber ].ulRegionSize        = 0x00000000;
		xMPUSettings->xRegion[ regionNumber ].ulRegionAttribute   = 0UL;

		/* If it's the current task, Invalidate it right away */
		if(isCurrentTask)
		{
			dsb();
			mtcp(XREG_CP15_MPU_MEMORY_REG_NUMBER, 12 + regionNumber);
			isb();
			mtcp(XREG_CP15_MPU_REG_BASEADDR,0x00000000);
			mtcp(XREG_CP15_MPU_REG_ACCESS_CTRL,0x00000000);
			mtcp(XREG_CP15_MPU_REG_SIZE_EN,0UL);
			dsb();
			isb();						/* synchronize context on this processor */
		}
			
	}
	vPortExitCritical();

	vPortResetPrivilege( xRunningPrivileged );
}


/*
 * Return to user mode if xRunningPrivileged is true. This function will be used in pair
 * with xPortRaisePrivilege. 
 */
void vPortResetPrivilege( BaseType_t wasUserMode )
{
	if( wasUserMode == pdTRUE )
	{
		//xil_printf("Task lowered priviledge\r\n");
		__asm__ __volatile__ ("CPS %0" :: "i"(XREG_CPSR_USER_MODE):"memory");
	}
	else
	{
		//xil_printf("Task didn't had to lower it's priviledge priviledge\r\n");
	}

}

/*
 * Check if the current mode is user mode. If so, make a SVC call to place us in System mode and return true. 
 * If the processor is already in any other priviledged mode, return false
 */
BaseType_t xPortRaisePrivilege( void )
{
	BaseType_t wasUserMode = pdFALSE;

	if ((mfcpsr() & XREG_CPSR_MODE_BITS) == XREG_CPSR_USER_MODE)
	{
		__asm__ __volatile__("SWI 1"::: "memory");
		 
		//xil_printf("Task asked and raised its priviledge\r\n");
		wasUserMode = pdTRUE;
	}
	else
	{
		//xil_printf("Task asked and didn't need to raised its priviledge\r\n");
		wasUserMode = pdFALSE;
	}

	return wasUserMode;
}

const char* xPortGetCPUModeStr(void)
{
	switch(mfcpsr() & XREG_CPSR_MODE_BITS)
	{
	case XREG_CPSR_USER_MODE:
		return "USER_MODE";

	case XREG_CPSR_SYSTEM_MODE:
		return "SYSTEM_MODE";

	case XREG_CPSR_IRQ_MODE:
		return "IRQ_MODE";

	case XREG_CPSR_SVC_MODE:
		return "SVC_MODE";

	case XREG_CPSR_UNDEFINED_MODE :
		return "UNDEFINED_MODE";

	case XREG_CPSR_DATA_ABORT_MODE:
		return "ABORT_MODE";

	case XREG_CPSR_FIQ_MODE:
		return "FIQ_MODE";

	default: return "NO";
	}
}


__attribute__((weak)) void setupMPU(void)
{
    xil_printf("MPU Regions were left as they were in mpu.c\r\n");
}



__attribute__((weak)) void vSVCOutOfRangeHandler(void)
{
    xil_printf("SVC number out of range\r\n");
}
/*-----------------------------------------------------------*/

