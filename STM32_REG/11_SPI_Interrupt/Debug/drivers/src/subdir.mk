################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32l4xx_gpio_driver.c \
../drivers/src/stm32l4xx_i2c_driver.c \
../drivers/src/stm32l4xx_spi_driver.c \
../drivers/src/stm32l4xx_usart_driver.c 

OBJS += \
./drivers/src/stm32l4xx_gpio_driver.o \
./drivers/src/stm32l4xx_i2c_driver.o \
./drivers/src/stm32l4xx_spi_driver.o \
./drivers/src/stm32l4xx_usart_driver.o 

C_DEPS += \
./drivers/src/stm32l4xx_gpio_driver.d \
./drivers/src/stm32l4xx_i2c_driver.d \
./drivers/src/stm32l4xx_spi_driver.d \
./drivers/src/stm32l4xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4 -DSTM32 -DNUCLEO_L476RG -DSTM32L476RGTx -c -I../Inc -I"C:/Users/USER/STM32CubeIDE/workspace_1.19.0/spi_interrupt/drivers/inc" -I"C:/Users/USER/STM32CubeIDE/workspace_1.19.0/spi_interrupt/drivers/src" -I"C:/Users/USER/STM32CubeIDE/workspace_1.19.0/spi_interrupt/drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32l4xx_gpio_driver.cyclo ./drivers/src/stm32l4xx_gpio_driver.d ./drivers/src/stm32l4xx_gpio_driver.o ./drivers/src/stm32l4xx_gpio_driver.su ./drivers/src/stm32l4xx_i2c_driver.cyclo ./drivers/src/stm32l4xx_i2c_driver.d ./drivers/src/stm32l4xx_i2c_driver.o ./drivers/src/stm32l4xx_i2c_driver.su ./drivers/src/stm32l4xx_spi_driver.cyclo ./drivers/src/stm32l4xx_spi_driver.d ./drivers/src/stm32l4xx_spi_driver.o ./drivers/src/stm32l4xx_spi_driver.su ./drivers/src/stm32l4xx_usart_driver.cyclo ./drivers/src/stm32l4xx_usart_driver.d ./drivers/src/stm32l4xx_usart_driver.o ./drivers/src/stm32l4xx_usart_driver.su

.PHONY: clean-drivers-2f-src

