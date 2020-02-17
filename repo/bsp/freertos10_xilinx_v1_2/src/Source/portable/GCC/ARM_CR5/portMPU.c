#include "FreeRTOS.h"

#if( portUSING_MPU_WRAPPERS == 1 )

	void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t ulStackDepth ){
		#warning "Complete this"
	}



/*
 * Return to user mode if xRunningPrivileged is true. This function will be used in pair
 * with xPortRaisePrivilege. 
 */
void vPortResetPrivilege( BaseType_t xRunningPrivileged )
{
	if( xRunningPrivileged != pdTRUE )
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
	 if ((mfcpsr() & XREG_CPSR_MODE_BITS) == XREG_CPSR_USER_MODE){
		 __asm__ __volatile__("SWI %0 ":: "i"(portSVC_RAISE_PRIVILEGE): "memory");
		 return pdTRUE;
	 }
	 else{
		 return pdFALSE;
	 }
}

#endif
