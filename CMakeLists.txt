cmake_minimum_required(VERSION 3.0.2)
project(ess_imu_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Specify compiler options
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# Default to C99
if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_C_FLAGS "-std=gnu99 ${CMAKE_C_FLAGS}")
    endif ()
else ()
    set (CMAKE_C_STANDARD 99)
endif ()

#######################################################
# The user only needs to select the PLATFORM and
# INTERFACE macro variables
#
# 1. Select PLATFORM type (NONE (PC) or RPI (RaspberryPi))
# 2. Select Serial INTERFACE type to UART or SPI
#
# NOTE: PLATFORM type must be set to RaspberryPi to support SPI interface

set(PLATFORM "NONE")
#set(PLATFORM "RPI")

set(INTERFACE "UART")
#set(INTERFACE "SPI")
#######################################################

message("---- Building for platform: ${PLATFORM}")
message("---- Building for interface: ${INTERFACE}")

# When SPI interface selected, mandatory platform is RPI
if (INTERFACE STREQUAL "SPI")
  set(PLATFORM "RPI")
  message("---- SPI selected forcing platform: ${PLATFORM}")
endif()

# Define macros variables for compilation
add_definitions(-D${INTERFACE})
add_definitions(-D${PLATFORM})

# Create file list for C libraries based on platform
if (PLATFORM STREQUAL "RPI")
  set(lib_sources
    src/hcl_rpi.c
    src/hcl_gpio_rpi.c
  )
elseif (PLATFORM STREQUAL "NONE")
  set(lib_sources
    src/hcl_linux.c
    src/hcl_gpio.c
  )
else()
  message([FATAL_ERROR] "**** Invalid Platform")
endif()


# Create file list for C libraries based on interface
if (INTERFACE STREQUAL "SPI")
  set(lib_sources ${lib_sources}
    src/hcl_spi_rpi.c
    src/sensor_epsonCommon.c
    src/sensor_epsonSpi.c
  )
elseif (INTERFACE STREQUAL "UART")
  set(lib_sources ${lib_sources}
    src/hcl_uart.c
    src/sensor_epsonCommon.c
    src/sensor_epsonUart.c
  )
else()
  message([FATAL_ERROR] "**** Invalid Interface")
endif()


# Declare a library for Epson IMU functions from C sources
add_library(ess_imu_driver_lib
  ${lib_sources}
)

# Link external libraries to Epson IMU Library
if (PLATFORM STREQUAL "RPI")
  # Determine location of wiringPi library on the host system
  # Needed if building on Raspberry Pi platform
  find_library(wiringPi_LIB NAMES wiringPi)

  target_link_libraries(ess_imu_driver_lib
    ${wiringPi_LIB}
  )
endif()


# Declare a C++ executable
if (INTERFACE STREQUAL "SPI")
add_executable(ess_imu_driver_node
  src/epson_imu_spi_ros_node.cpp
)
elseif (INTERFACE STREQUAL "UART")
add_executable(ess_imu_driver_node
  src/epson_imu_uart_ros_node.cpp
)
else()
  message([FATAL_ERROR] "**** Invalid Interface")
endif()

# Link Epson IMU library to executable target ROS node
target_link_libraries(ess_imu_driver_node
  ${catkin_LIBRARIES}
  ess_imu_driver_lib
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ess_imu_driver_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
