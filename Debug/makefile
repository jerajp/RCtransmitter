################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
RCtransmitter.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RCtransmitter.list \

OBJCOPY_BIN += \
RCtransmitter.bin \


# All Target
all: RCtransmitter.elf secondary-outputs

# Tool invocations
RCtransmitter.elf: $(OBJS) $(USER_OBJS) /home/primoz/MEGA/github/RCtransmitter/STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "RCtransmitter.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/home/primoz/MEGA/github/RCtransmitter/STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RCtransmitter.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RCtransmitter.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RCtransmitter.list"
	@echo 'Finished building: $@'
	@echo ' '

RCtransmitter.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RCtransmitter.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
