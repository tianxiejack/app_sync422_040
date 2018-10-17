################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/SPI/gpio_rdwr.cpp \
../src/SPI/spidev_trans.cpp \
../src/SPI/sync422_trans.cpp \
../src/SPI/vidScheduler.cpp 

OBJS += \
./src/SPI/gpio_rdwr.o \
./src/SPI/spidev_trans.o \
./src/SPI/sync422_trans.o \
./src/SPI/vidScheduler.o 

CPP_DEPS += \
./src/SPI/gpio_rdwr.d \
./src/SPI/spidev_trans.d \
./src/SPI/sync422_trans.d \
./src/SPI/vidScheduler.d 


# Each subdirectory must supply rules for building sources it contributes
src/SPI/%.o: ../src/SPI/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -I../include -I../src/OSA_CAP/inc -I/usr/lib/aarch64-linux-gnu/include -O3 -ccbin aarch64-linux-gnu-g++ -gencode arch=compute_20,code=sm_20 -m64 -odir "src/SPI" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -I../include -I../src/OSA_CAP/inc -I/usr/lib/aarch64-linux-gnu/include -O3 --compile -m64 -ccbin aarch64-linux-gnu-g++  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


