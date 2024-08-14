################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1220/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 --vcu_support=vcrc -O0 --include_path="C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2" --include_path="C:/ti/c2000/C2000Ware_4_03_00_00" --include_path="C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2/device" --include_path="C:/ti/c2000/C2000Ware_4_03_00_00/driverlib/f2838x/driverlib" --include_path="C:/ti/ccs1220/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --define=_FLASH --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2/CPU1_FLASH/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1846533681: ../c2000.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1220/ccs/utils/sysconfig_1.15.0/sysconfig_cli.bat" -s "C:/ti/c2000/C2000Ware_4_03_00_00/.metadata/sdk.json" -d "F2838x" --script "C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2/c2000.syscfg" -o "syscfg" --package 337bga --part F2838x_337bga --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/board.c: build-1846533681 ../c2000.syscfg
syscfg/board.h: build-1846533681
syscfg/board.cmd.genlibs: build-1846533681
syscfg/board.opt: build-1846533681
syscfg/pinmux.csv: build-1846533681
syscfg/c2000ware_libraries.cmd.genlibs: build-1846533681
syscfg/c2000ware_libraries.opt: build-1846533681
syscfg/c2000ware_libraries.c: build-1846533681
syscfg/c2000ware_libraries.h: build-1846533681
syscfg/clocktree.h: build-1846533681
syscfg/: build-1846533681

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1220/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 --vcu_support=vcrc -O0 --include_path="C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2" --include_path="C:/ti/c2000/C2000Ware_4_03_00_00" --include_path="C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2/device" --include_path="C:/ti/c2000/C2000Ware_4_03_00_00/driverlib/f2838x/driverlib" --include_path="C:/ti/ccs1220/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --define=_FLASH --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="C:/Users/BharatBohara/workspace_v12/tiCANJ1939_v2/CPU1_FLASH/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


