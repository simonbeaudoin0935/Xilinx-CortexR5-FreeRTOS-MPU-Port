#include "FreeRTOS.h"
#include "task.h"

#if( portUSING_MPU_WRAPPERS == 1 )



void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t ulStackDepth )
{
    extern uint32_t __privileged_data_start[];
    extern uint32_t __privileged_data_end[];
    extern uint32_t __privileged_functions_start[];
    extern uint32_t __privileged_functions_end[];

    extern uint32_t __psu_r5_ddr_0_MEM_0_start;
    extern uint32_t __psu_r5_ddr_0_MEM_0_size;

    if( xRegions == NULL )
	{
	    /* No MPU regions are specified so allow access to all RAM. */
		xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress = __psu_r5_ddr_0_MEM_0_start;

		xMPUSettings->xRegion[ 0 ].ulRegionSize = __psu_r5_ddr_0_MEM_0_size;

        xMPUSettings->xRegion[ 0 ].ulRegionAttribute = portMPU_REGION_READ_WRITE | portMPU_REGION_CACHEABLE_BUFFERABLE | portMPU_REGION_ENABLE;
			

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

    	/* This function is called automatically when the task is created - in
		which case the stack region parameters will be valid.  At all other
		times the stack parameters will not be valid and it is assumed that the
		stack region has already been configured. */
		if( ulStackDepth > 0 )
		{
			/* Define the region that allows access to the stack. */
			xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress = ( uint32_t ) pxBottomOfStack;
        
            xMPUSettings->xRegion[ 0 ].ulRegionSize = ulStackDepth;

			xMPUSettings->xRegion[ 0 ].ulRegionAttribute = portMPU_REGION_READ_WRITE | portMPU_REGION_CACHEABLE_BUFFERABLE | portMPU_REGION_ENABLE ;
		}

		size_t lIndex = 0;

		for( size_t i = 1; i <= portNUM_CONFIGURABLE_REGIONS; i++ )
		{
			if( ( xRegions[ lIndex ] ).ulLengthInBytes > 0UL )
			{
				xMPUSettings->xRegion[ i ].ulRegionBaseAddress = ( uint32_t ) xRegions[ lIndex ].pvBaseAddress;
                xMPUSettings->xRegion[ i ].ulRegionSize        = ( uint32_t ) xRegions[ lIndex ].ulLengthInBytes;
				xMPUSettings->xRegion[ i ].ulRegionAttribute   = xRegions[lIndex].ulParameters | portMPU_REGION_ENABLE;

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
	__asm__ __volatile__ ("CPS %0" :: "i"(XREG_CPSR_USER_MODE):"memory");
	}
}

/*
 * Check if the current mode is user mode. If so, make a SVC call to place us in System mode and return true. 
 * If the processor is already in any other priviledged mode, return false
 */
BaseType_t xPortRaisePrivilege( void )
{
    BaseType_t wasUserMode = pdFALSE;

	 if ((mfcpsr() & XREG_CPSR_MODE_BITS) == XREG_CPSR_USER_MODE){
		 __asm__ __volatile__("SWI %0 ":: "i"(portSVC_RAISE_PRIVILEGE): "memory");
		 return pdTRUE;
	 }
	 else{
		 return pdFALSE;
	 }
}

void setupMPU(void)
{

}

#endif
