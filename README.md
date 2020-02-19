# Xilinx CortexR5 FreeRTOS MPU-Port for version 2018.3

The ARM Cortex R5 processor is a mighty powerfull processor. It lies between a fully equipped ARM CortexA* with MPU support for running complex OS like linux, and smaller CortexM4 which don't have cache support nor branch prediction. To be fully capable, the Cortex R5 lacks pretty much only an MPU. But to deal with virtual memory was not the intended goal. Instead, the CR5 features a well features MPU. Despite its name; Memory Protection Unit, the MPU not only exposes a mechanism to protect regions of memory, it specifies cache and memory space behavior for memory regions (ie. device type, strongly ordered, normal memory, etc). 

The FreeRTOS os is an excellent choice for embedded developpement, and it is the officialy supportes RTOS for CortexR5 when developping for Xilinx's ZynqMP SoCs. FreeRTOS features in it's API functions that use MPU capabilities to help protect the developper against him and hard to find bugs related to memory management issues. Unfortunately, the CortexR5 lacks official port of FreeRTOS+MPU, even though the cpu has an MPU. This is very frustrating since developping for zynq chips comes with a great deal of challenges already regarding many aspecs, including memory management ans sharing accross several CPUs and FPGA.
Since Xilinx didn't bothered developping the missing code in its provision of FreeRTOS, I decided to attempt to port it. The initiative came from personal frustration when working on a large project targeting all cores (RPU0, RPU1 and APU) with several developpers. People don't always have the same experience and backgroud in embedded developpment, which means that when thousands of lines of code begin to pile up, and the number of tasks grows large, a single mistake in one task ends up ruining everyone's day. It often takes several context switches until the CPU eventually goes rogue, which makes debugging a complete nightmare, even when good tracing is active.

# Repo structure
in this repo you will find the 'repo' folder which contains the 'bsp' folder, essentially making it a Xilinx SDK repository folder. When in a XSDK workspace, the fact of proviting the folder as a repo will do that the freertos port will have preseence on the standard one. 

There is also the 'exemple_workspace' folder which, like it's name implies, is a ready to generate a workspace to test the features of freertos+mpu. Open the Xilinx Software Command Line Tool 2018.3 and navigate to this folder. Then, execute the tcl script like this : 'source gen_workspace.tcl'. It will create the workspace based on the 'system.hdf' file present in the folder. By default, it's a Ultra96 hardware, with default presets and no bistream. Simply replace it with your's, clear the workspace and regenerate it. 

# Details worth mentionning

This port is still a work in progress. Any contribution to make it more clean/complete/robust will be greatly appreciates.

The function 'setupMPU' was left as a week symbol in the bsp in order for the user to provide their own. The port will use internally the MPU regions 12, 13, 14 and 15. Region #12 is used for the task stack coverage. Regions 13,14 and 15 are used for the 3 use definable regions. Official FreeRTOS always talks about user being able to provide 3 regions, so I left it like that. That mean the use can safely configure region 0 trough 11 in setupMPU. 




