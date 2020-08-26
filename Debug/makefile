################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/subdir.mk
-include startup/subdir.mk
-include source/subdir.mk
-include drivers/subdir.mk
-include board/src/subdir.mk
-include board/subdir.mk
-include CMSIS/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SD2_TP3_ALLIONE_PINCIROLI.axf

# Tool invocations
SD2_TP3_ALLIONE_PINCIROLI.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -Xlinker -Map="SD2_TP3_ALLIONE_PINCIROLI.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -mcpu=cortex-m0plus -mthumb -T "SD2_TP3_ALLIONE_PINCIROLI_Debug.ld" -o "SD2_TP3_ALLIONE_PINCIROLI.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) SD2_TP3_ALLIONE_PINCIROLI.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "SD2_TP3_ALLIONE_PINCIROLI.axf"; # arm-none-eabi-objcopy -v -O binary "SD2_TP3_ALLIONE_PINCIROLI.axf" "SD2_TP3_ALLIONE_PINCIROLI.bin" ; # checksum -p MKL46Z256xxx4 -d "SD2_TP3_ALLIONE_PINCIROLI.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
