#include "xil_mpu.h"
#include "xil_printf.h"
#include "portMPU.h"


/* setup MPU is declared as a weak function in bsp, which means that this one will have preseence over it. */
void setupMPU(void){
	xil_printf("MPU setup!\r\n");

	size_t regnum = 10;

	extern uint32_t __privileged_functions_start[];
	extern uint32_t __privileged_functions_end[];
	extern uint32_t __privileged_data_start[];
	extern uint32_t __privileged_data_end[];

	xil_printf("__privileged_functions_start : 0x%08X\r\n", __privileged_functions_start);
	xil_printf("__privileged_functions_end   : 0x%08X\r\n", __privileged_functions_end);

	xil_printf("__privileged_data_start : 0x%08X\r\n", __privileged_data_start);
	xil_printf("__privileged_data_end   : 0x%08X\r\n", __privileged_data_end);

	Xil_SetMPURegionByRegNum(regnum++,
							(INTPTR) __privileged_functions_start,
							__privileged_functions_end - __privileged_functions_start,
							portMPU_REGION_PRIVILEGED_READ_ONLY | portMPU_REGION_CACHEABLE_BUFFERABLE);

	Xil_SetMPURegionByRegNum(regnum++,
							(INTPTR) __privileged_data_start,
							__privileged_data_end - __privileged_data_start,
							portMPU_REGION_PRIVILEGED_READ_WRITE | portMPU_REGION_CACHEABLE_BUFFERABLE | portMPU_REGION_EXECUTE_NEVER);


	/* This leaves region 12, 13, 14, 15 for user  */
}


