#include "xil_mpu.h"
#include "xil_printf.h"
#include "portMPU.h"


/* setup MPU is declared as a weak function in bsp, which means that this one will have preseence over it. */
void setupMPU(void){

	extern uint32_t __privileged_functions_start__[];
	extern uint32_t __privileged_functions_end__[];

	extern uint32_t __privileged_data_start__[];
	extern uint32_t __privileged_data_end__[];

	extern uint32_t __psu_r5_ddr_0_MEM_0_start__[];
	extern uint32_t __psu_r5_ddr_0_MEM_0_end__[];


	INTPTR __psu_r5_ddr_0_MEM_0_size__      = (INTPTR)__psu_r5_ddr_0_MEM_0_end__      - (INTPTR)__psu_r5_ddr_0_MEM_0_start__;
	INTPTR __privileged_functions_size__ = (INTPTR)__privileged_functions_end__ - (INTPTR)__privileged_functions_start__;
	INTPTR __privileged_data_size__      = (INTPTR)__privileged_data_end__      - (INTPTR)__privileged_data_start__;


	xil_printf("__privileged_functions_start : 0x%08X\r\n", __privileged_functions_start__);
	xil_printf("__privileged_functions_end   : 0x%08X\r\n", __privileged_functions_end__);
	xil_printf("__privileged_functions_size  : 0x%08X\r\n", __privileged_functions_size__);

	xil_printf("__privileged_data_start : 0x%08X\r\n", __privileged_data_start__);
	xil_printf("__privileged_data_end   : 0x%08X\r\n", __privileged_data_end__);
	xil_printf("__privileged_data_size  : 0x%08X\r\n", __privileged_data_size__);


	Xil_DisableMPU();

	/* Privileged functions are read only for privileged */
	Xil_DisableMPURegionByRegNum(10);
	Xil_SetMPURegionByRegNum(10,
							(INTPTR) __privileged_functions_start__,
							__privileged_functions_size__,
							PRIV_RO_USER_NA | NORM_NSHARED_WB_WA);

	/* Privileged data is read write for privileged */
	Xil_DisableMPURegionByRegNum(11);
	Xil_SetMPURegionByRegNum(11,
							(INTPTR) __privileged_data_start__,
							__privileged_data_size__,
							PRIV_RW_USER_NA | NORM_NSHARED_WB_WA | EXECUTE_NEVER);

	Xil_EnableMPU();

	return;

	/* Mark whole address space as non accessible */
	Xil_DisableMPURegionByRegNum(0);
	Xil_SetMPURegionByRegNum(0,
							 (INTPTR)0x00000000,
							 0x100000000, //REGION_4G
							 NO_ACCESS);

	/* Section : ATCM
	 * Region  : 0x00000000 - 0x0000FFFF
	 * Size    : 0x00010000 (exactlty REGION_64K)
	 * Cache   : Normal memory, non shared, write-back cacheable, Write allocated
	 * Access  : Full Access
	 * Comment : ACTM memory is multiplexed over the same DDR region
	 */
	Xil_DisableMPURegionByRegNum(1);
	Xil_SetMPURegionByRegNum(1,
							(INTPTR)0x00000000,
							0x00010000,
							NORM_NSHARED_WB_WA | PRIV_RW_USER_RO);

	/* Section : BTCM
	 * Region  : 0x00020000 - 0x0002FFFF
	 * Size    : 0x00010000 (exactlty REGION_64K)
	 * Cache   : Normal memory, non shared, write-back cacheable, Write allocated
	 * Access  : Full Access
	 * Comment : BCTM memory is multiplexed over the same DDR region
	 */
	Xil_DisableMPURegionByRegNum(2);
	Xil_SetMPURegionByRegNum(2,
							 (INTPTR)0x00020000,
							 0x00010000,
							 NORM_NSHARED_WB_WA | PRIV_RW_USER_RO);



	/* 16M of device memory from 0xF8000000 to 0xF8FFFFFF for STM_CORESIGHT */
	Xil_DisableMPURegionByRegNum(3);
	Xil_SetMPURegionByRegNum(3,
							 0xF8000000U,
							 0x1000000,  //REGION_16M
							 DEVICE_NONSHARED | PRIV_RW_USER_RW);

	/* 1M of device memory from 0xF9000000 to 0xF90FFFFF for RPU_A53_GIC */
	Xil_DisableMPURegionByRegNum(4);
	Xil_SetMPURegionByRegNum(4,
							 0xF9000000U,
							 0x100000,  //REGION_1M
							 DEVICE_NONSHARED | PRIV_RW_USER_RW);

	/* 16M of device memory from 0xFE000000 to 0xFEFFFFFF for Upper LPS slaves */
	Xil_DisableMPURegionByRegNum(5);
	Xil_SetMPURegionByRegNum(5,
							 0xFE000000U,
							 0x1000000,  //REGION_16M
							 DEVICE_NONSHARED | PRIV_RW_USER_RW);


	/*
	 * 16M of device memory from 0xFF000000 to 0xFFFFFFFF for Lower LPS slaves,
	 * CSU, PMU, TCM, OCM
	 */
	Xil_DisableMPURegionByRegNum(6);
	Xil_SetMPURegionByRegNum(6,
							 0xFF000000U,
							 0x1000000,  //REGION_16M
							 DEVICE_NONSHARED | PRIV_RW_USER_RW);

	/* 256K of OCM RAM from 0xFFFC0000 to 0xFFFFFFFF marked as normal memory */
	Xil_DisableMPURegionByRegNum(7);
	Xil_SetMPURegionByRegNum(7,
							 0xFFFC0000U,
							 0x40000,  //REGION_256K
							 NORM_NSHARED_WB_WA| PRIV_RW_USER_RW);


	/* Whole ddr reserved segment is read only for user space */
	Xil_DisableMPURegionByRegNum(8);
	Xil_SetMPURegionByRegNum(8,
							 (INTPTR)__psu_r5_ddr_0_MEM_0_start__,
							 __psu_r5_ddr_0_MEM_0_size__,
							 NORM_NSHARED_WB_WA | PRIV_RW_USER_RO);

	/* Privileged functions are read only for privileged */
	Xil_DisableMPURegionByRegNum(9);
	Xil_SetMPURegionByRegNum(9,
							(INTPTR) __privileged_functions_start__,
							__privileged_functions_size__,
							PRIV_RO_USER_NA | NORM_NSHARED_WB_WA);

	/* Privileged data is read write for privileged */
	Xil_DisableMPURegionByRegNum(10);
	Xil_SetMPURegionByRegNum(10,
							(INTPTR) __privileged_data_start__,
							__privileged_data_size__,
							PRIV_RW_USER_NA | NORM_NSHARED_WB_WA | EXECUTE_NEVER);

	Xil_EnableMPU();
	/* This leaves region 11, 12, 13, 14, 15 for user  */
}


