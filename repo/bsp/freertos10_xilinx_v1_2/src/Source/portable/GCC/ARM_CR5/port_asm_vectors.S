/*
 * FreeRTOS Kernel V10.0.0
 * Copyright (C) 2014 - 2018 Xilinx, Inc. All rights reserved.
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


.org 0
.text

.globl _boot

.globl _vector_table
.globl FIQInterrupt
.globl FreeRTOS_IRQ_Handler
.globl FreeRTOS_SWI_Handler
.globl DataAbortInterrupt
.globl PrefetchAbortInterrupt

.section .vectors,"a"
_vector_table:
	ldr	pc,=_boot
	ldr	pc,=UndefinedHandler
	ldr   pc, _swi
	ldr	pc,=PrefetchAbortHandler
	ldr	pc,=DataAbortHandler
	NOP	/* Placeholder for address exception vector*/
	ldr   pc, _irq
	ldr	pc,=FIQHandler

_irq:   .word FreeRTOS_IRQ_Handler
_swi:   .word FreeRTOS_SWI_Handler

.text
FIQHandler:					/* FIQ vector handler */
	stmdb	sp!,{r0-r3,r12,lr}		/* state save from compiled code */
FIQLoop:
	bl	FIQInterrupt			/* FIQ vector */
	ldmia	sp!,{r0-r3,r12,lr}		/* state restore from compiled code */
	subs	pc, lr, #4			/* adjust return */

/*
Undefined:					// Undefined handler 
	stmdb	sp!,{r0-r3,r12,lr}		// state save from compiled code 
	ldr     r0, =UndefinedExceptionAddr
	sub     r1, lr, #4
	str     r1, [r0]            		// Store address of instruction causing undefined exception 

	bl	UndefinedException		// UndefinedException: call C function here 
	ldmia	sp!,{r0-r3,r12,lr}		// state restore from compiled code
	movs	pc, lr
*/

/*
DataAbortHandler:				/// Data Abort handler
	tmdb	sp!,{r0-r3,r12,lr}		// state save from compiled code
	ldr     r0, =DataAbortAddr
	sub     r1, lr, #8
	str     r1, [r0]            		// Stores instruction causing prefetch abort
	bl	DataAbortInterrupt		// PrefetchAbortInterrupt: call C function here
	ldmia	sp!,{r0-r3,r12,lr}		// state restore from compiled code
	subs	pc, lr, #4
*/

/*
PrefetchAbortHandler:				// Prefetch Abort handler
	stmdb	sp!,{r0-r3,r12,lr}		// state save from compiled code
	ldr     r0, =PrefetchAbortAddr
	sub     r1, lr, #4
	str     r1, [r0]            		// Stores instruction causing prefetch abort
	bl	PrefetchAbortInterrupt		// PrefetchAbortInterrupt: call C function here
	ldmia	sp!,{r0-r3,r12,lr}		// state restore from compiled code
	movs	pc, lr
*/

.end


