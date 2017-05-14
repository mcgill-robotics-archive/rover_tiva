cmake_minimum_required(VERSION 2.8.3)
project(arm_wrist)

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
add_definitions(-DROSSERIAL_BAUDRATE=57600)
add_definitions(-DTX_BUFFER_SIZE=256)
add_definitions(-DRX_BUFFER_SIZE=256)

add_definitions(-DARM_WRIST)

# Allow multiple defs for compiling usb_serial_structs.c
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z muldefs")

# Generate target for TivaC
generate_tivac_firmware(
  USB
  SRCS arm_wrist.cpp usb_serial_structs.c ../lib/bdc_motor/bdc_motor.cpp ../lib/inc_encoder/inc_encoder.cpp
  INCS . ../lib/
  BOARD tm4c123gxl
  DEVICE_SILICON TARGET_IS_TM4C123_RB2
)
