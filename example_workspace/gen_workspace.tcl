
set SDK_path "/opt/Xilinx/SDK/2018.3"

# Create workspace
set current_path [exec pwd]

#set workspace
setws -switch "$current_path"

#Add the repository containing freertos bsp
repo -set "../repo"

#Create HW project based on ZCU102 template, which is fine for exmple purpose
createhw -name ZCU102_hw_platform -hwspec $SDK_path/data/embeddedsw/lib/hwplatform_templates/ZCU102_hw_platform/system.hdf


#create BSPs for RPU0
createbsp -name RPU0_bsp -proc psu_cortexr5_0 -hwproject ZCU102_hw_platform -os freertos10_xilinx

#Configure the bsp for using mpu 
configbsp -bsp RPU0_bsp use_mpu_support true

#Regenerate both BSPs 
regenbsp -bsp RPU0_bsp


#import projects
importprojects "$current_path/RPU0_MPU_Example"

#Make sure it's in debug config by default
configapp -app RPU0_MPU_Example build-config Debug

