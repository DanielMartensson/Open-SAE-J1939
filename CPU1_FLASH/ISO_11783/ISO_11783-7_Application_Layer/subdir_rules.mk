################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ISO_11783/ISO_11783-7_Application_Layer/%.obj: ../ISO_11783/ISO_11783-7_Application_Layer/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1220/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 --vcu_support=vcrc -O0 --include_path="C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2" --include_path="C:/ti/c2000/C2000Ware_4_03_00_00" --include_path="C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2/device" --include_path="C:/ti/c2000/C2000Ware_4_03_00_00/driverlib/f2838x/driverlib" --include_path="C:/ti/ccs1220/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --define=_FLASH --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="ISO_11783/ISO_11783-7_Application_Layer/$(basename $(<F)).d_raw" --include_path="C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2/CPU1_FLASH/syscfg" --obj_directory="ISO_11783/ISO_11783-7_Application_Layer" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


