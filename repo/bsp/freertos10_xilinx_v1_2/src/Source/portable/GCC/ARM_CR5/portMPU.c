#include "FreeRTOS.h"
#include "task.h"
#include "xpseudo_asm_gcc.h"
#include "xreg_cortexr5.h"
#include "xil_printf.h"

#if( portUSING_MPU_WRAPPERS == 1 )

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
    uint32_t Regionsize;

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
    /* First region is stack, always fill it. */
	xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress = pxBottomOfStack;

	xMPUSettings->xRegion[ 0 ].ulRegionSize = (getSizeEncoding(ulStackDepth) << 1) | REGION_EN;

	xMPUSettings->xRegion[ 0 ].ulRegionAttribute = PRIV_RW_USER_RW | NORM_NSHARED_WB_WA | EXECUTE_NEVER;

    if( xRegions == NULL )
	{
		/* Invalidate all other regions. */
		for( size_t i = 1; i <= portNUM_CONFIGURABLE_REGIONS; i++ )
		{
		    xMPUSettings->xRegion[ i ].ulRegionBaseAddress = 0x00000000;
    	    xMPUSettings->xRegion[ i ].ulRegionSize = 0x00000000;
		    xMPUSettings->xRegion[ i ].ulRegionAttribute = 0UL;
		}
	}
    else
    {
		size_t lIndex = 0;

		for( size_t i = 1; i <= portNUM_CONFIGURABLE_REGIONS; i++ )
		{
			if( ( xRegions[ lIndex ] ).ulLengthInBytes > 0UL )
			{
				xMPUSettings->xRegion[ i ].ulRegionBaseAddress = ( uint32_t ) xRegions[ lIndex ].pvBaseAddress;
                xMPUSettings->xRegion[ i ].ulRegionSize        = (getSizeEncoding(( uint32_t ) xRegions[ lIndex ].ulLengthInBytes) << 1 )| REGION_EN;
				xMPUSettings->xRegion[ i ].ulRegionAttribute   = xRegions[lIndex].ulParameters;

			}
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
		return "RQ_MODE";

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

#endif
