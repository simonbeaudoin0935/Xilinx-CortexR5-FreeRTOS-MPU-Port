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

	.text
	.arm

	.set SYS_MODE,	0x1f
	.set SVC_MODE,	0x13
	.set IRQ_MODE,	0x12

    .set MAX_SVC,	2

	/* Hardware registers. */
	.extern ulICCIAR
	.extern ulICCEOIR
	.extern ulICCPMR

	/* Variables and functions. */
	.extern ulMaxAPIPriorityMask
	.extern _freertos_vector_table
	.extern pxCurrentTCB
	.extern vTaskSwitchContext
	.extern vApplicationIRQHandler
	.extern ulPortInterruptNesting
	.extern ulPortTaskHasFPUContext

	.global FreeRTOS_IRQ_Handler
	.global FreeRTOS_SWI_Handler
	.global vPortRestoreTaskContext

.macro portSAVE_CONTEXT

	/* Save the LR and SPSR onto the system mode stack before switching to
	system mode to save the remaining system mode registers. */
	SRSDB	sp!, #SYS_MODE
	CPS		#SYS_MODE
	PUSH	{R0-R12, R14}

	/* Push the critical nesting count. */
	LDR		R2, ulCriticalNestingConst
	LDR		R1, [R2]
	PUSH	{R1}

	/* Does the task have a floating point context that needs saving?  If
	ulPortTaskHasFPUContext is 0 then no. */
	LDR		R2, ulPortTaskHasFPUContextConst
	LDR		R3, [R2]
	CMP		R3, #0

	/* Save the floating point context, if any. */
	FMRXNE  R1,  FPSCR
	VPUSHNE {D0-D15}
	/*VPUSHNE	{D16-D31}*/
	PUSHNE	{R1}

	/* Save ulPortTaskHasFPUContext itself. */
	PUSH	{R3}

	/* Save the stack pointer in the TCB. */
	LDR		R0, pxCurrentTCBConst
	LDR		R1, [R0]
	STR		SP, [R1]

	.endm

; /**********************************************************************/

.macro portRESTORE_CONTEXT

	/* Set the SP to point to the stack of the task being restored. */
	LDR		R0, pxCurrentTCBConst
	LDR		R1, [R0]
	LDR		SP, [R1]





//**************************************************************

    /* We program the register number 12*/
    MOV     R0, #12
    MCR     p15, 0, R0, c6,  c2, 0 //MPU_MEMORY_REG_NUMBER
    ISB

    /* Program the basse address of the stack */
    ADD     R1, R1, #4
    LDR     R0, [R1]
    MCR     p15, 0, R0, c6,  c1, 0 //MPU_REG_BASEADDR

    /* Program the access control of the stack */
    ADD     R1, R1, #4
    LDR     R0, [R1]  
    MCR     p15, 0, R0, c6,  c1, 4 //MPU_REG_ACCESS_CTRL


    /* Program the size of the stack */
    ADD     R1, R1, #4
    LDR     R0, [R1]
    MCR     p15, 0, R0, c6,  c1, 2 //MPU_REG_SIZE_EN



//**************************************************************

    /* We program the register number 13*/
    MOV     R0, #13
    MCR     p15, 0, R0, c6,  c2, 0 //MPU_MEMORY_REG_NUMBER
    ISB

    /* Program the basse address of region 1 */
    ADD     R1, R1, #4
    LDR     R0, [R1]
    MCR     p15, 0, R0, c6,  c1, 0 //MPU_REG_BASEADDR

    /* Program the access control of region 1 */
    ADD     R1, R1, #4
    LDR     R0, [R1]  
    MCR     p15, 0, R0, c6,  c1, 4 //MPU_REG_ACCESS_CTRL

    /* Program the size of region 1*/
    ADD     R1, R1, #4
    LDR     R0, [R1]
    MCR     p15, 0, R0, c6,  c1, 2 //MPU_REG_SIZE_EN




//**************************************************************

    /* We program the register number 14*/
    MOV     R0, #14
    MCR     p15, 0, R0, c6,  c2, 0 //MPU_MEMORY_REG_NUMBER
    ISB

    /* Program the basse address of region 2 */
    ADD     R1, R1, #4
    LDR     R0, [R1]
    MCR     p15, 0, R0, c6,  c1, 0 //MPU_REG_BASEADDR

    /* Program the access control of region 2 */
    ADD     R1, R1, #4
    LDR     R0, [R1]  
    MCR     p15, 0, R0, c6,  c1, 4 //MPU_REG_ACCESS_CTRL

    /* Program the size of region 2*/
    ADD     R1, R1, #4
    LDR     R0, [R1]
    MCR     p15, 0, R0, c6,  c1, 2 //MPU_REG_SIZE_EN







    /* We program the register number 15*/
    MOV     R0, #15
    MCR     p15, 0, R0, c6,  c2, 0 //MPU_MEMORY_REG_NUMBER
    ISB

    /* Program the basse address of region 3 */
    ADD     R1, R1, #4
    LDR     R0, [R1]
    MCR     p15, 0, R0, c6,  c1, 0 //MPU_REG_BASEADDR

    /* Program the access control of region 3 */
    ADD     R1, R1, #4
    LDR     R0, [R1]  
    MCR     p15, 0, R0, c6,  c1, 4 //MPU_REG_ACCESS_CTRL

    /* Program the size of region 3*/
    ADD     R1, R1, #4
    LDR     R0, [R1]
    MCR     p15, 0, R0, c6,  c1, 2 //MPU_REG_SIZE_EN


//**************************************************************




	/* Is there a floating point context to restore?  If the restored
	ulPortTaskHasFPUContext is zero then no. */
	LDR		R0, ulPortTaskHasFPUContextConst
	POP		{R1}
	STR		R1, [R0]
	CMP		R1, #0

	/* Restore the floating point context, if any. */
	POPNE 	{R0}
	/*VPOPNE	{D16-D31}*/
	VPOPNE	{D0-D15}
	VMSRNE  FPSCR, R0

	/* Restore the critical section nesting depth. */
	LDR		R0, ulCriticalNestingConst
	POP		{R1}
	STR		R1, [R0]

	/* Ensure the priority mask is correct for the critical nesting depth. */
	LDR		R2, ulICCPMRConst
	LDR		R2, [R2]
	CMP		R1, #0
	MOVEQ	R4, #255
	LDRNE	R4, ulMaxAPIPriorityMaskConst
	LDRNE	R4, [R4]
	STR		R4, [R2]

    
	/* Restore all system mode registers other than the SP (which is already
	being used). */
	POP		{R0-R12, R14}

	/* Return to the task code, loading CPSR on the way. */
	RFEIA	sp!

	.endm




/******************************************************************************
 * SVC handler is used to start the scheduler.
 *****************************************************************************/
.align 4
.type FreeRTOS_SWI_Handler, %function
FreeRTOS_SWI_Handler:

    PUSH    {R0-R12, LR}            /* Store registers */
    LDR     R0, [lr, #-4]           /* Calculate address of SVC instruction */
                                    /* and load it into R0. */
    BIC     R0, R0, #0xFF000000     /* Mask off top 8 bits of instruction */
                                    /* to give SVC number. */

    CMP     R0, #MAX_SVC            /* Range check */
    LDRLS   PC, [PC, R0, LSL #2]    /* If the range is ok, load address of svc code from jump table below*/
    B       .SVCOutOfRange           /* If landing here, svc number was out of range */

.SVCJumpTable:    
    .word     .SVCnum0
    .word     .SVCnum1
 
.SVCnum0:                             /* SVC #1 is a context switch request */
    POP    {R0-R12, LR}             /* Put back the registers as they were entering the svc */
	portSAVE_CONTEXT                /* Save the context */
	LDR R0, vTaskSwitchContextConst /* load the address of the switch task c function */
	BLX	R0                          /* Execute that function */
	portRESTORE_CONTEXT             /* Switch to the context of that task */

.SVCnum1:                             /* SVC #0 is a privilege elevation request */
    MRS     R0, SPSR                /* Load the saved CPSR into R0 */
    ORR     R0, R0, #SYS_MODE       /* Set the mode bits for system mode in spsr */
    MSR     SPSR, R0                /* Put back the altered spsr in it's register*/
    POP     {R0-R12, LR}            /* Put back the registers as they were entering the svc */
    MOVS    PC, LR                  /* Return from SVC. Puts back cpsr */  



.SVCOutOfRange:

	LDR		R0, vSVCOutOfRangeHandlerConst
	BLX		R0




/******************************************************************************
 * vPortRestoreTaskContext is used to start the scheduler.
 *****************************************************************************/
.type vPortRestoreTaskContext, %function
vPortRestoreTaskContext:
	/* Switch to system mode. */
	CPS		#SYS_MODE
	portRESTORE_CONTEXT



.align 4
.type FreeRTOS_IRQ_Handler, %function
FreeRTOS_IRQ_Handler:

	/* Return to the interrupted instruction. */
	SUB		lr, lr, #4

	/* Push the return address and SPSR. */
	PUSH	{lr}
	MRS		lr, SPSR
	PUSH	{lr}

	/* Change to supervisor mode to allow reentry. */
	CPS		#SVC_MODE

	/* Push used registers. */
	PUSH	{r0-r4, r12}

	/* Increment nesting count.  r3 holds the address of ulPortInterruptNesting
	for future use.  r1 holds the original ulPortInterruptNesting value for
	future use. */
	LDR		r3, ulPortInterruptNestingConst
	LDR		r1, [r3]
	ADD		r4, r1, #1
	STR		r4, [r3]

	/* Read value from the interrupt acknowledge register, which is stored in r0
	for future parameter and interrupt clearing use. */
	LDR 	r2, ulICCIARConst
	LDR		r2, [r2]
	LDR		r0, [r2]

	/* Ensure bit 2 of the stack pointer is clear.  r2 holds the bit 2 value for
	future use.  _RB_ Is this ever needed provided the start of the stack is
	alligned on an 8-byte boundary? */
	MOV		r2, sp
	AND		r2, r2, #4
	SUB		sp, sp, r2

	/* Call the interrupt handler. */
	PUSH	{r0-r4, lr}
	LDR		r1, vApplicationIRQHandlerConst
	BLX		r1
	POP		{r0-r4, lr}
	ADD		sp, sp, r2

	CPSID	i
	DSB
	ISB

	/* Write the value read from ICCIAR to ICCEOIR. */
	LDR 	r4, ulICCEOIRConst
	LDR		r4, [r4]
	STR		r0, [r4]

	/* Restore the old nesting count. */
	STR		r1, [r3]

	/* A context switch is never performed if the nesting count is not 0. */
	CMP		r1, #0
	BNE		exit_without_switch

	/* Did the interrupt request a context switch?  r1 holds the address of
	ulPortYieldRequired and r0 the value of ulPortYieldRequired for future
	use. */
	LDR		r1, =ulPortYieldRequired
	LDR		r0, [r1]
	CMP		r0, #0
	BNE		switch_before_exit

exit_without_switch:
	/* No context switch.  Restore used registers, LR_irq and SPSR before
	returning. */
	POP		{r0-r4, r12}
	CPS		#IRQ_MODE
	POP		{LR}
	MSR		SPSR_cxsf, LR
	POP		{LR}
	MOVS	PC, LR

switch_before_exit:
	/* A context swtich is to be performed.  Clear the context switch pending
	flag. */
	MOV		r0, #0
	STR		r0, [r1]

	/* Restore used registers, LR-irq and SPSR before saving the context
	to the task stack. */
	POP		{r0-r4, r12}
	CPS		#IRQ_MODE
	POP		{LR}
	MSR		SPSR_cxsf, LR
	POP		{LR}
	portSAVE_CONTEXT

	/* Call the function that selects the new task to execute.
	vTaskSwitchContext() if vTaskSwitchContext() uses LDRD or STRD
	instructions, or 8 byte aligned stack allocated data.  LR does not need
	saving as a new LR will be loaded by portRESTORE_CONTEXT anyway. */
	LDR		R0, vTaskSwitchContextConst
	BLX		R0

	/* Restore the context of, and branch to, the task selected to execute
	next. */
	portRESTORE_CONTEXT

ulICCIARConst:	.word ulICCIAR
ulICCEOIRConst:	.word ulICCEOIR
ulICCPMRConst: .word ulICCPMR
pxCurrentTCBConst: .word pxCurrentTCB
ulCriticalNestingConst: .word ulCriticalNesting
ulPortTaskHasFPUContextConst: .word ulPortTaskHasFPUContext
ulMaxAPIPriorityMaskConst: .word ulMaxAPIPriorityMask
vTaskSwitchContextConst: .word vTaskSwitchContext
vApplicationIRQHandlerConst: .word vApplicationIRQHandler
ulPortInterruptNestingConst: .word ulPortInterruptNesting
vSVCOutOfRangeHandlerConst: .word vSVCOutOfRangeHandler

.end
