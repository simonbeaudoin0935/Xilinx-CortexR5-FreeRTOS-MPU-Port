#include "FreeRTOS.h"
#include "task.h"
#include "xpseudo_asm_gcc.h"
#include "xreg_cortexr5.h"

#if( portUSING_MPU_WRAPPERS == 1 )



void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t ulStackDepth )
{
    /* First region is stack, always fill it. */
	xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress = pxBottomOfStack;

	xMPUSettings->xRegion[ 0 ].ulRegionSize = ulStackDepth;

	xMPUSettings->xRegion[ 0 ].ulRegionAttribute = PRIV_RW_USER_RW | NORM_NSHARED_WB_WA | EXECUTE_NEVER

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
                xMPUSettings->xRegion[ i ].ulRegionSize        = ( uint32_t ) xRegions[ lIndex ].ulLengthInBytes;
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
		__asm__ __volatile__("SWI %0 ":: "i"(portSVC_RAISE_PRIVILEGE): "memory");
		 
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


void setupMPU(void)
{

}

#endif
