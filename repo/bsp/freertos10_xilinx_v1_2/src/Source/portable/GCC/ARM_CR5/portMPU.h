#ifndef PORTMPU_H
#define PORTMPU_H

#ifdef __cplusplus
	extern "C" {
#endif

/* BSP includes. */
#include "xil_types.h"
#include "xpseudo_asm.h"
#include "xreg_cortexr5.h"

/* Used in stack initialization */
#define portINITIAL_CONTROL_IF_UNPRIVILEGED		( 0x03 )
#define portINITIAL_CONTROL_IF_PRIVILEGED		( 0x02 )


#define portPRIVILEGE_BIT			( 0x80000000UL )


//Cortex R5 MPU masks, defined in xreg_cortexr5.h
#define portMPU_REGION_READ_WRITE								PRIV_RW_USER_RW
#define portMPU_REGION_PRIVILEGED_READ_ONLY						PRIV_RO_USER_NA
#define portMPU_REGION_READ_ONLY								PRIV_RO_USER_RO
#define portMPU_REGION_PRIVILEGED_READ_WRITE					PRIV_RW_USER_NA
#define portMPU_REGION_PRIVILEGED_READ_WRITE_UNPRIV_READ_ONLY	PRIV_RW_USER_RO
#define portMPU_REGION_CACHEABLE_BUFFERABLE						NORM_SHARED_WB_WA
#define portMPU_REGION_EXECUTE_NEVER							EXECUTE_NEVER



#define portUNPRIVILEGED_FLASH_REGION		( 0UL )
#define portPRIVILEGED_FLASH_REGION			( 1UL )
#define portPRIVILEGED_RAM_REGION			( 2UL )
#define portGENERAL_PERIPHERALS_REGION		( 3UL )
#define portSTACK_REGION					( 4UL )
#define portFIRST_CONFIGURABLE_REGION	    ( 5UL )
#define portLAST_CONFIGURABLE_REGION		( 7UL )
#define portNUM_CONFIGURABLE_REGIONS		( ( portLAST_CONFIGURABLE_REGION - portFIRST_CONFIGURABLE_REGION ) + 1 )
#define portTOTAL_NUM_REGIONS				( portNUM_CONFIGURABLE_REGIONS + 1 ) /* Plus one to make space for the stack region. */


typedef struct MPU_REGION_REGISTERS
{
	uint32_t ulRegionBaseAddress;
	uint32_t ulRegionAttribute;
} xMPU_REGION_REGISTERS;

/* Plus 1 to create space for the stack region. */
typedef struct MPU_SETTINGS
{
	xMPU_REGION_REGISTERS xRegion[ portTOTAL_NUM_REGIONS ];
} xMPU_SETTINGS;

#warning "Check numbers here for swi"

/* SVC numbers for various services. */
#define portSVC_START_SCHEDULER				0
#define portSVC_YIELD						1
#define portSVC_RAISE_PRIVILEGE				2

#warning "check the portNVIC thing"
//#define portYIELD_WITHIN_API() 													\
//{																				\
//	/* Set a PendSV to request a context switch. */								\
//	/*portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;*/							\
//																				\
//	/* Barriers are normally not required but do ensure the code is completely	\
//	within the specified behaviour for the architecture. */						\
//	__asm volatile( "dsb" ::: "memory" );										\
//	__asm volatile( "isb" );													\
//}

void vPortResetPrivilege( BaseType_t xRunningPrivileged );

BaseType_t xPortRaisePrivilege( void );



#ifdef __cplusplus
	} /* extern C */
#endif

#endif /* PORTMPU_H */
