
# Create workspace
set current_path [exec pwd]

#set workspace
setws -switch "$current_path"

#Add the repository containing freertos bsp
repo -set "../repo"


#Create HW project
#createhw -name HW -hwspec "system.hdf"


#create BSPs for RPU0 and RPU1
createbsp -name RPU0_bsp -proc psu_cortexr5_0 -hwproject ZCU102_hw_platform -os freertos10_xilinx


#Regenerate both BSPs 
regenbsp -bsp RPU0_bsp


#import projects
importprojects "$current_path/RPU0_MPU_Example"

configapp -app RPU0_MPU_Example build-config Debug

