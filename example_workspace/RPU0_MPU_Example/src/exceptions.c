#include "xil_printf.h"
#include "xreg_cortexr5.h"
#include "xpseudo_asm_gcc.h"
//#include "task.h"

uint32_t UndefinedExceptionAddress;
uint32_t DataAbortAddress;
uint32_t PrefetchAbortAddress;



uint32_t r[16];



void PrintAuxiliaryFaultStatusRegister(uint32_t auxReg){

	xil_printf("\t\tAUXILIARY FAULT STATUS REGISTER INFORMATION\r\n\n");
	xil_printf("\t\tAuxiliary data fault register : 0x%08x\r\n", auxReg);
	xil_printf("\t\tIndex value for the access giving the error : %d\r\n\n", (auxReg >> 5)& 0x000001FF);
	xil_printf("\t\t**This field is only valid for data cache store parity/ECC errors. On the AIFSR, and for TCM accesses, this field SBZ\r\n\n");
	xil_printf("\t\tSource of the error : ");
	uint32_t tmp = (auxReg >> 22) & 0x00000003;
	tmp |= ((auxReg >> 18) & 0x00000004);
	switch(tmp){
	case 0b000: xil_printf("Cache/AXIM\r\n\n");
		break;
	case 0b001: xil_printf("ATCM\r\n\n");
		break;
	case 0b010: xil_printf("BTCM\r\n\n");
		break;
	case 0b011:
	case 0b100: xil_printf("Reserved\r\n\n");
		break;
	case 0b101: xil_printf("AXI peripheral port, including virtual interface\r\n\n");
		break;
	case 0b110: xil_printf("AHB peripheral port\r\n\n");
		break;
	case 0b111: xil_printf("Reserved\r\n\n");
		break;
	default:
		break;
	}
	if(auxReg & 0x00200000){
		xil_printf("\t\tRecoverable error : Recoverable error. This includes all correctable parity/ECC errors and recoverable TCM external errors\r\n\n");
	}else{
		xil_printf("\t\tRecoverable error : Unrecoverable error.\r\n\n");
	}

	xil_printf("\t\tCache way or ways in which the error occurred : %d\r\n\n", ((auxReg >> 24) & 0x0F000000));
	xil_printf("\t\t*This field is only valid for data cache store parity/ECC errors, otherwise it is Unpredictable.\r\n\n");
}

void PrintFaultStatusExplanation(uint32_t FaultStatus, uint32_t auxiliaryFault)
{
	uint32_t tmp = FaultStatus & 0x00000400;
	tmp >>= 6;
	tmp = tmp | (FaultStatus & 0x0000000F);

	switch(tmp){
	case 0b00001:
	 	xil_printf("\t-> ALIGNMENT\r\n\n");
	   	xil_printf("\t\tAn alignment fault is generated if a data access is performed to an address that is not aligned for\r\n"
	   			   "\t\tthe size of the access, and strict alignment is required for the access. A number of instructions\r\n"
	   			   "\t\tthat access memory, for example, LDM and STC, require strict alignment. See the ARM Architecture\r\n"
	   			   "\t\tReference Manual for more information. In addition, strict alignment can be required for all data\r\n"
	   			   "\t\taccesses by setting the A-bit in the SCTLR\r\n");
	   	break;

	case 0b00000:
	   	xil_printf("\t-> BACKGROUND\r\n\n");
	   	xil_printf("\t\tFrom ARM infocenter : A background fault is generated when the MPU is enabled and a memory access\r\n"
	   			   "\t\tis made to an address that is not within an enabled subregion of an MPU region. A background fault\r\n"
	   			   "\t\tdoes not occur if the background region is enabled and the access is Privileged.\r\n");
	   	break;

	case 0b01101:
	   	xil_printf("\t-> PERMISSION\r\n\n");
	   	xil_printf("\t\tA permission fault is generated when a memory access does not meet the requirements of the permissions\r\n"
	   			   "\t\tdefined for the memory region that it accesses.\r\n");
	   	break;

	case 0b01000:
	   	xil_printf("\t-> SYNCHRONOUS EXTERNAL ABORT\r\n");
	   	break;

	case 0b10110:
	   	xil_printf("\t-> ASYNCHRONOUS EXTERNAL ABORT\r\n");
	   	xil_printf("\t***For Asynchronous events, the fault address register is unpredictable.\r\n\n");
	   	break;

	case 0b11001:
	  	xil_printf("\t-> SYNCHRONOUS PARITY OR ECC ERROR\r\n");
	   	PrintAuxiliaryFaultStatusRegister(auxiliaryFault);
	   	break;

	case 0b11000:
	   	xil_printf("\t-> ASYNCHRONOUS PARITY OR ECC ERROR\r\n");
	   	xil_printf("\t***For Asynchronous events, the fault address register is unpredictable.\r\n\n");
	   	PrintAuxiliaryFaultStatusRegister(auxiliaryFault);
	   	break;

	case 0b00010:
	   	//xil_printf("DEBUG EVENT\r\n");
	   	break;

	default:
	   	xil_printf("Unrecognised fault status\r\n");
	};
}

void UndefinedInterruptHandler(void *InstancePtr)
{

	xil_printf("\n\n\r********** Undefined Exception on RPU0 *********************\n\n\r");

	xil_printf("\tThe processor takes the Undefined Instruction exception when:\r\n");
	xil_printf("\t\t- A double-precision VFP operation is attempted when only single-precision support is implemented.\r\n");
	xil_printf("\t\t- a VFP operation is attempted when the VFP is not enabled.\r\n");
	xil_printf("\t\t- When a UDIV or SDIV instruction is executed, the value in Rm is zero, and the DZ bit in the SCTLR is set.\r\n");
	xil_printf("\tOr, the instruction was faulty.Possible reasons for the execution of a faulty instruction : \r\n");
	xil_printf("\t\t- Branch to RAM code that has been corrupted or not yet initialized with required functions.\r\n");
	xil_printf("\t\t- Corrupted branch address.\r\n");
	xil_printf("\t\t\tA) Trying to execute code in the wrong instruction set or Thumb branch inside a 32-bit Thumb instruction.\r\n");
	xil_printf("\t\t\tB) Branch to non-code area or inline literals.\r\n");
	xil_printf("\t\t\tC) Return address on the stack has been corrupted (for example, stack overflow or pop/push count mismatch).\r\n");
	xil_printf("\t\t\tD) Function pointers not initialized or corrupted.\r\n");
	xil_printf("\t\t- Issue with interworking (Thumb > Arm, Arm > Thumb)\r\n");


	xil_printf("\n\n\tAddress of the undefined instruction %lx\r\n",UndefinedExceptionAddress);


    xil_printf("Press 'x' to resume execution past the faulty instruction ...\r\n");

	while(1){
        if('x' == inbyte()) break;
    }
}

void PrefetchAbortInterruptHandler(void *InstancePtr)
{
	uint32_t instFaultStatus    = mfcp(XREG_CP15_INST_FAULT_STATUS);
	uint32_t auxInstFaultStatus = mfcp(XREG_CP15_AUX_INST_FAULT_STATUS);
	uint32_t instFaultAddress   = mfcp(XREG_CP15_INST_FAULT_ADDRESS);

	xil_printf("\n\n\r********** Prefetch Abort Exception on RPU0 ****************\n\n\r");

	xil_printf("\tInstruction Fault Status Register : 0x%08x\r\n\n",instFaultStatus);

	xil_printf("\tInstruction Fault Address Register : 0x%08x\r\n\n", instFaultAddress);

	xil_printf("\tThe name of the task that was currently executing : %s\r\n\n",pcTaskGetName(NULL));

	xil_printf("\tAddress of Instruction causing Prefetch abort 0x%08x\r\n\n",PrefetchAbortAddress);

	PrintFaultStatusExplanation(instFaultStatus,auxInstFaultStatus);

	//exception_core_dump();

	uint32_t* stack_pointer = (uint32_t*) r[13];

	xil_printf("\tStack dump :\r\n\n");

	for(size_t i = 0; i != 20; i++){

		xil_printf("\t\tstack[%d] : %08x\r\n",i,*stack_pointer);

		stack_pointer++;
	}

	xil_printf("\tPress 'x' to resume execution past the faulty instruction ...\r\n");

	while(1){
        if('x' == inbyte()) break;
    }
}

void DataAbortInterruptHandler(void *InstancePtr)
{
	uint32_t dataFaultStatus    = mfcp(XREG_CP15_DATA_FAULT_STATUS);
	uint32_t auxDataFaultStatus = mfcp(XREG_CP15_AUX_DATA_FAULT_STATUS);
	uint32_t dataFaultAddress   = mfcp(XREG_CP15_DATA_FAULT_ADDRESS);

	xil_printf("\n\n\r**********Data Abort Exception on RPU0 *******************\n\n\r");

    xil_printf("\tData Fault Status Register : 0x%08x\r\n",dataFaultStatus);

    xil_printf("\tData Fault Address Register : 0x%08x\r\n", dataFaultAddress);

    xil_printf("\tThe name of the task that was currently executing : %s\n\r",pcTaskGetName(NULL));

    if(dataFaultStatus & 0x00000800){
    	xil_printf("\r\n\tWrite access caused the abort.\r\n\n");
    }else{
    	xil_printf("\r\n\tRead access caused the abort.\r\n\n");
    }

    PrintFaultStatusExplanation(dataFaultStatus, auxDataFaultStatus);

	xil_printf("\r\n\tAddress of Instruction causing Data abort 0x%p\r\n",DataAbortAddress);


	//exception_core_dump();


	xil_printf("\tStack dump :\r\n\n");

	uint32_t* stack_pointer = (uint32_t*) r[13];

	for(size_t i = 0; i != 20; i++){

		xil_printf("\t\tstack[%d] : %08x\r\n",i,*stack_pointer);

		stack_pointer++;
	}

    xil_printf("\r\n\n\tPress 'x' to resume execution past the faulty instruction or 'r' to retry it ...\r\n\n");


	while(1){

        if('x' == inbyte()) break;
    }
}


void __attribute__((__naked__)) UndefinedHandler(void){

	__asm volatile("stmdb	sp!,{r0-r3,r12,lr}             \n" /* state save from compiled code*/ \
				   "ldr     r0, =UndefinedExceptionAddress \n" /* branch */ \
	               "sub     r1, lr, #4                     \n" \
				   "str     r1, [r0]            		   \n"); /* Store address of instruction causing undefined exception */ \

    UndefinedInterruptHandler(0);

	__asm volatile("ldmia	sp!,{r0-r3,r12,lr}           \n" /* state restore from compiled code */ \
				   "movs	pc, lr                       \n");

}


void __attribute__((__naked__)) PrefetchAbortHandler(void){

	/* Save context for display */
	__asm volatile("PUSH    {R0}                       \n" /* Push R0 on the stack because we are going to use it */ \
				   "MOV		R0, %[regs_array]          \n" /* Put the address of the first register of the cpu registers array, plus 1 (r[1]) */ \
				   "ADD     R0, R0, #4                 \n" /* Compute address of R1 register in the cpu registers array */ \
				   "STMIA   R0!, {R1-R12}               \n" /* Store registers R1-R12 in the array */\
				   "POP     {R0}                       \n" /* Now we will save R0 too, so pop its previous value from the stack */\
				   "PUSH    {R1}                       \n" /* We are now going to use R1, like we used R0, so push it on the stack */\
				   "MOV     R1, %[regs_array]          \n" /* Put the address of the first register of the cpu registers array (r[0]) */\
				   "STR		R0, [R1]                   \n" /* Store R0 */\
				   "CPS		#0x1F                      \n" /* Put ourselves in system mode to access banked version of SP and LR */\
				   "ADD     R1, R1, #13*4              \n" /* Compute the address of the SP in the array */\
				   "STR     SP, [R1]                   \n" /* Store system-mode SP in the cpu registers array */\
				   "ADD     R1, R1, #4                 \n" /* Compute the address of the LR register in the array */\
				   "STR     LR, [R1]                   \n" /* Store system-mode LR in the cpu registers array */\
				   "CPS		#0x17                      \n" /* Return to abort mode */\
				   "LDR     R1, =PrefetchAbortAddress  \n" /* Load data abort address faulty instruction*/ \
				   "PUSH    {R0}                       \n" /* Push R0 again because we are going to use it */ \
				   "SUB     R0, LR, #4                 \n" /* Substract 8 to the link register to obtain the address of faulty instruction */ \
				   "STR     R0, [R1]                   \n" /* Stores instruction causing data abort */ \
				   "POP     {R0}                       \n" /* Pop back R1 because we are done with it */\
				   "POP     {R1}                       \n" /* Pop back R1 because we are done with it */\
				   "STMDB	SP!, {R0-R3,R12,LR}        \n" /* Now prodece to standard push of R0-R3,R12,LR on the stack to be able to call procedures that use prologues*/\
				   : /* Empty output */                     \
				   : [regs_array] "r" (r) \
				   :								        \
			);


	PrefetchAbortInterruptHandler(0);

	__asm volatile("LDMIA	SP!,{R0-R3,R12,LR}         \n"    \
				   "MOVS	PC, LR                     \n");
}

void __attribute__((__naked__)) DataAbortHandler(void){

	__asm volatile(
			"	PUSH    {R0}                       \n" /* Push R0 on the stack because we are going to use it */
			"	MOV		R0, %[regs_array]          \n" /* Put the address of the first register of the cpu registers array, plus 1 (r[1]) */
			"	ADD     R0, R0, #4                 \n" /* Compute address of R1 register in the cpu registers array */
			"	STMIA   R0!, {R1-R12}               \n" /* Store registers R1-R12 in the array */\
			"	POP     {R0}                       \n" /* Now we will save R0 too, so pop its previous value from the stack */\
			"	PUSH    {R1}                       \n" /* We are now going to use R1, like we used R0, so push it on the stack */\
			"	MOV     R1, %[regs_array]          \n" /* Put the address of the first register of the cpu registers array (r[0]) */\
			"	STR		R0, [R1]                   \n" /* Store R0 */\
			"	CPS		#0x1F                      \n" /* Put ourselves in system mode to access banked version of SP and LR */\
			"	ADD     R1, R1, #13*4              \n" /* Compute the address of the SP in the array */\
			"	STR     SP, [R1]                   \n" /* Store system-mode SP in the cpu registers array */\
			"	ADD     R1, R1, #4                 \n" /* Compute the address of the LR register in the array */\
			"	STR     LR, [R1]                   \n" /* Store system-mode LR in the cpu registers array */\
			"	CPS		#0x17                      \n" /* Return to abort mode */\
			"	LDR     R1, =DataAbortAddress      \n" /* Load data abort address faulty instruction*/
			"	PUSH    {R0}                       \n" /* Push R0 again because we are going to use it */
			"	SUB     R0, LR, #8                 \n" /* Substract 8 to the link register to obtain the address of faulty instruction */
			"	STR     R0, [R1]                   \n" /* Stores instruction causing data abort */
			"	POP     {R0}                       \n" /* Pop back R1 because we are done with it */\
			"	POP     {R1}                       \n" /* Pop back R1 because we are done with it */\
			"	STMDB	SP!, {R0-R3,R12,LR}        \n" /* Now prodece to standard push of R0-R3,R12,LR on the stack to be able to call procedures that use prologues*/\
			: /* Empty output */
			: [regs_array] "r" (r)
			:
		);

	DataAbortInterruptHandler(0);

	__asm volatile("LDMIA	SP!,{R0-R3,R12,LR}\n"    \
				   "SUBS	PC, LR, #4\n");
}
