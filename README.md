# README for Epson IMU Driver for ROS1 Node

<!---toc start-->

- [README for Epson IMU Driver for ROS1 node](#readme-for-epson-imu-driver-for-ros1-node)
  - [What is this repository for?](#what-is-this-repository-for)
  - [What kind of hardware or software will I likely need?](#what-kind-of-hardware-or-software-will-i-likely-need)
    - [For the UART Interface:](#for-the-uart-interface)
    - [For the SPI Interface:](#for-the-spi-interface)
  - [How do I use the driver?](#how-do-i-use-the-driver)
  - [How do I use the driver if usleep() is not supported for time delays?](#how-do-i-use-the-driver-if-usleep-is-not-supported-for-time-delays)
  - [How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?](#how-do-i-use-the-driver-with-gpios-to-control-imu-reset-drdy-ext-pins)
  - [How do I build, install, run this package?](#how-do-i-build-install-run-this-package)
    - [Example console output of *catkin_make* build:](#example-console-output-of-catkin_make-build)
    - [Example console output of launching ROS node:](#example-console-output-of-launching-ros-node)
  - [What does this ROS IMU Node publish as messages?](#what-does-this-ros-imu-node-publish-as-messages)
    - [ROS Topic Message /imu/data](#ros-topic-message-imudata)
    - [ROS Topic Message /imu/data_raw](#ros-topic-message-imudata_raw)
  - [Why am I seeing inaccurate ROS timestamps, high latencies, or slower than expected IMU data rates?](#why-am-i-seeing-inaccurate-ros-timestamps-high-latencies-or-slower-than-expected-imu-data-rates)
    - [ROS timestamps](#ros-timestamps)
    - [When using USB-UART bridges](#when-using-usb-uart-bridges)
      - [Modifying *latency_timer* by udev mechanism](#modifying-latency_timer-by-udev-mechanism)
      - [Modifying *latency_timer* by sysfs mechanism](#modifying-latency_timer-by-sysfs-mechanism)
      - [Modifying *low_latency* flag using *setserial* utility](#modifying-low_latency-flag-using-setserial-utility)
    - [When using the SPI interface](#when-using-the-spi-interface)
  - [Package Contents](#package-contents)
  - [References](#references)

<!---toc end-->

## What is this repository for?

- This code is a ROS1 package for demonstrating a ROS node that configures and publishes IMU messages from a supported Epson IMU.
- This code provides software communication between Epson IMU and ROS using the either the UART or SPI interface.
- For the UART connection, this code uses the standard Unix Terminal I/O library (termios) for communicating either direct or by USB-serial converter such as FTDI USB-UART bridge ICs.
- For the SPI connection, this code uses the [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/) library for accessing GPIO and SPI functions on the Raspberry Pi platform running Ubuntu Linux distro.
- This ROS1 node demonstration software is a ROS C++ wrapper around the Linux C driver software:
  - *src/epson_imu_uart_ros_node.cpp* is for the UART interface
  - *src/epson_imu_spi_ros_node.cpp* is for the SPI interface
- The other source files in `src/` are based on the Linux C driver originally released and can be found here:
  [Linux C driver and logger example for EPSON IMU](https://github.com/cubicleguy/imu_linux_example)
- Information about ROS, and tutorials can be found: [ROS.org](https://docs.ros.org/)

## What kind of hardware or software will I likely need?

- Epson IMU [Epson IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/)
  - At the time software release:
    - G320PDG0, G320PDGN, G354PDH0, G364PDCA, G364PDC0
    - G365PDC1, G365PDF1, G370PDF1, G370PDS0
    - G330PDG0, G366PDG0, G370PDG0, G370PDT0
    - G570PR20
- ROS Noetic (via download) [ROS.org](https://wiki.ros.org/noetic/Installation/Ubuntu)
- This software was developed and tested on the following:

```
  ROS1:        Noetic
  Description: Ubuntu 20.04 LTS
  Hardware Platform: Core i7 PC, Raspberry Pi 3B+, RaspberryPi 4
```

### For the UART Interface:

- Epson USB evaluation board or equivalent FTDI USB-Serial interface connecting the Epson IMU to ROS host (tty/serial) [See M-G32EV041](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
- Alternatively, a direct connection from the Epson IMU to the ROS platform supporting a 3.3V CMOS compatible UART interface [See M-G32EV031](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/).

### For the SPI Interface:

- **NOTE:** This software is intended for an embedded Linux host system with 3.3V I/O compatible SPI interface (SCLK, MISO, MOSI) and 3 GPIOs (CS#, RST#, DRDY).
- For Raspberry Pi, if not enabled already the SPI interface can be enabled using `raspi-config` or equivalent.
- This code uses a separate GPIO to manually control CS# chipselect instead of the chipselect assigned to by the RapsberryPi SPI interface.
  - The chipselect assigned by the HW SPI interface should also work, but has not been thoroughly tested.
- Epson Breakout evaluation board or some equivalent is required to connect to the 3.3V CMOS compatible pins of the ROS host (SPI & GPIOs) [See M-G32EV031](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)

## How do I use the driver?

- This code assumes that the user is familiar with building ROS packages using the `catkin_make` build process.

- This README is *NOT* detailed step by step instructions on how to build and install ROS software.

- Please refer to the ROS.org website for more detailed instructions on the ROS package build process. [ROS.org](https://wiki.ros.org/ROS/Tutorials)

- **NOTE:** At a bare minimum, the user must re-build the package with catkin_make after modifying the `CMakeLists.txt` to configure any of the following:

  - serial interface type, `INTERFACE=` (UART or SPI)
  - host platform type, `PLATFORM=` (RPI or NONE=PC)

- Changes to IMU settings should be made by editing the `.launch` files located in the `launch/` folder instead of modifying the `src/` source files

## How do I use the driver if usleep() is not supported for time delays?

- **NOTE:** In the `hcl_linux.c` or `hcl_rpi.c`, there are `seDelayMS()` and `seDelayMicroSecs()` wrapper functions for time delays in millisecond and microseconds, respectively.
- On embedded Linux platforms, the user may need to modify and redirect to platform specific delay routines if `usleep()` is not supported.
- For example on RaspberryPi, the time delay functions for millisecond and microseconds are redirected to WiringPi library `delay()` and `delayMicroseconds()`, respectively.
- If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.

## How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?

- When connecting the IMU using the UART interface, the use of GPIO pins for connecting to the IMU `RESET#` or `DRDY` is optional, but can be useful on an embedded Linux platforms (such as RapsberryPi).
- When connecting the IMU using the SPI interface, the use of GPIO pins for connecting to the IMU `DRDY` is mandatory (`RESET#` is recommended, `EXT` is optional).
- When using the `time_correction` function for accurate time stamping with an external 3.3V GNSS 1PPS signal, the IMU `EXT` pin must be connected to the 1PPS.
- The user can choose to control the IMU `CS#` pin directly with a designated host GPIO output or connect to the designated chipselect of the SPI interface (i.e. `SPI_CE0` pin on the RapsberryPi)
- Where possible, connecting the `RESET#` and forcing a Hardware Reset during every IMU initialization is recommended for better robustness.
- This code is structured to simplify editing to redirect GPIO control to low-level hardware library GPIO function calls.
- There are no standard methods to implement GPIO connections on embedded Linux platform, but the following source files typically need changing:

```
src/hcl_linux.c
src/hcl_gpio.c
src/hcl_gpio.h
```

- Typically, an external library needs to be invoked to initialize & enable GPIO HW functions on the user's embedded platform.

- This typically requires the following changes to `hcl_linux.c` (Refer to `hcl_rpi.c` as a template):

  - add `#include` to external library near the top of `hcl_linux.c`
  - add the initialization call to the HW library inside the `seInit()` function in `hcl_linux.c`

For example on a Raspberry Pi, the following changes can be made to `hcl_linux.c`:

```
...

  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library

  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");

    return OK;
  }

...
```

- Typically, the GPIO pins need to be assigned according to embedded HW platform-specific **pin numbering** and requires changes to `hcl_gpio.h`.

**NOTE:** When using the SPI interface, if direct `CS#` control is not by GPIO, then connect IMU `CS#` pin to the RPI `SPI0_CE0 (P1_24)`.

For example on a Raspberry Pi, the changes to `hcl_gpio.h` assume the following pin mapping:

```
Epson IMU                   Raspberry Pi
---------------------------------------------------
EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input
```

```
...

// Prototypes for generic GPIO functions
int gpioInit(void);
int gpioRelease(void);

void gpioSet(uint8_t pin);
void gpioClr(uint8_t pin);
uint8_t gpioGetPinLevel(uint8_t pin);

#define RPI_GPIO_P1_15              22                    // <== Added
#define RPI_GPIO_P1_18              24                    // <== Added

#define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
#define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
...
```

- The external HW library will typically have GPIO pin control functions such as `set_output()`, `set_input()`, `set()`, `reset()`, `read_pin_level()`, etc...

- The user should redirect the function calls in `hcl_gpio.c` for `gpioInit()`, `gpioRelease()`, `gpioSet()`, `gpioClr()`, `gpioGetPinLevel()` to such equivalent external HW library pin control functions.

- For example on a Raspberry Pi, the following are changes to `hcl_gpio.c` which can be used as a template:

```
#include "hcl.h"
#include "hcl_gpio.h"
#include <wiringPi.h>                         // <== Added external library

...

int gpioInit(void)
{
	pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
	pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
	pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

	return OK;
}

...

int gpioRelease(void)
{
	return OK;
}

...

void gpioSet(uint8_t pin)
{
	digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
}

...

void gpioClr(uint8_t pin)
{
	digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
}

...

uint8_t gpioGetPinLevel(uint8_t pin)
{
	return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
}

...
```

## How do I build, install, run this package?

The Epson IMU ROS driver is designed for building with the standard ROS catkin build environment.
Refer to the ROS1 Tutorials for more info: [ROS1 Tutorial](https://wiki.ros.org/ROS/Tutorials)

For more information on ROS & catkin setup refer to: [Installing and Configuring ROS Environment](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

1. Place this package (including folders) into a new folder within your catkin workspace `src/` folder.
   For example, we recommend using the folder name `ess_imu_driver`

```
<catkin_workspace>/src/ess_imu_driver/
```

2. Modify the `CMakeLists.txt` to select the serial interface type `INTERFACE=` and platform type `PLATFORM=` that matches your ROS system and connection to the Epson IMU.
   Refer to the comments in the `CMakeLists.txt` for additional info.
   **NOTE:** You *MUST* re-build using `catkin_make` after any changes in the `CMakeLists.txt`.

3. From the catkin workspace folder run `catkin_make` to build all ROS packages located in the `<catkin_workspace>/src/` folder.

   **NOTE:** Re-run `catkin_make` to rebuild the software after making any changes to any of the `.c` or `.cpp` or `.h` source files.

**NOTE:** It is recommended to change IMU settings by editing the parameters in the `.launch` launch file, instead of modifying the `.c`, `.cpp`, `.h` source files directly.

```
<catkin_workspace>/catkin_make
```

4. Reload the current ROS environment variables that may have changed after the catkin_make build process by entering the following from the `\<catkin_workspace>`:

```
source ./devel/setup.bash
```

5. Modify the `.launch` file in the launch/ folder to set your desired IMU configure parameter options at runtime:

| Launch Parameter | Comment                                                                                                                                                                     |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| serial_port      | **NOTE:** Only applicable when using UART interface to specify the port name, otherwise ignored ie. "/dev/ttyUSB0" or "/dev/ttyXXX"                                         |
| frame_id         | specifies the value in the IMU message frame_id field                                                                                                                       |
| ext_sel          | specifies the function of the GPIO2 pin (GPIO2, External Trigger, External Counter Reset). **NOTE:** Must be set to External Counter Reset when time_correction is enabled. |
| ext_pol          | specifies the polarity of the GPIO2 pin when *ext_sel* is External Trigger or External Counter Reset **NOTE:** Only applicable to G320/G354/G364/V340 otherwise ignored.    |
| dout_rate        | specifies the IMU output data rate                                                                                                                                          |
| filter_sel       | specifies the IMU filter setting                                                                                                                                            |
| flag_out         | specifies to enable or disable ND_FLAG status in IMU output data (not used by ROS)                                                                                          |
| temp_out         | specifies to enable or disable TempC sensor in IMU output data (**must be enabled**)                                                                                        |
| gyro_out         | specifies to enable or disable Gyro sensor in IMU output data (**must be enabled**)                                                                                         |
| accel_out        | specifies to enable or disable Accl sensor in IMU output data (**must be enabled**)                                                                                         |
| qtn_out          | specifies to enable or disable Quaternion in IMU output data (**only for models that support quaternion output, otherwise ignored**)                                        |
| count_out        | specifies to enable or disable counter in IMU output data (**must be enabled when time_correction is enabled**)                                                             |
| checksum_out     | specifies to enable or disable checksum in IMU output data (not used by ROS, but checksum errors are detected when enabled)                                                 |
| atti_profile     | specifies the attitude motion profile (**only for models that support quaternion output, otherwise ignored**)                                                               |
| time_correction  | enables time correction function using External Counter Reset function & external 1PPS connected to IMU GPIO2/EXT pin. **NOTE:** Must set count_out=1 & ext_sel=1           |

**NOTE:** The ROS launch file passes IMU configuration settings to the IMU at runtime. Therefore, rebuilding with `catkin_make` is not required.

6. Start the Epson IMU ROS driver use the appropriate `.launch` file located in `launch/`. All parameters are described in the in-line comments of the `.launch` file.

For example, to launch the ROS node:

```
<catkin_workspace>/roslaunch ess_imu_driver normal.launch
```

| Launch File   | Description                                                                                                                                                                         |
| ------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| normal.launch | Publishes to `imu/data` for IMU models that support `gyro`, `accel`, and `quaternion (orientation)` or `imu/data_raw` for IMU models that do not support `quaternion (orientation)` |
| raw.launch    | Publishes to `imu/data_raw` and does not enable or update `quaternion (orientation)`                                                                                                |
| tc.launch     | Same as normal.launch except time correction function is enabled (GNSS 1PPS signal must be connected to IMU EXT pin)                                                                |

### Example console output of *catkin_make* build

```
user@user:~/catkin_ws$ catkin_make
Base path: /home/user/catkin_ws
Source space: /home/user/catkin_ws/src
Build space: /home/user/catkin_ws/build
Devel space: /home/user/catkin_ws/devel
Install space: /home/user/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/user/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/user/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/user/catkin_ws/devel;/opt/ros/noetic
-- This workspace overlays: /home/user/catkin_ws/devel;/opt/ros/noetic
-- Found PythonInterp: /bin/python3 (found suitable version "3.8.10", minimum required is "3")
-- Using PYTHON_EXECUTABLE: /bin/python3
-- Using Debian Python package layout
-- Using empy: /usr/lib/python3/dist-packages/em.py
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/user/catkin_ws/build/test_results
-- Forcing gtest/gmock from source, though one was otherwise available.
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /bin/python3 (found version "3.8.10")
-- Using Python nosetests: /bin/nosetests3
-- catkin 0.8.10
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - ess_imu_driver
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'ess_imu_driver'
-- ==> add_subdirectory(ess_imu_driver)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- Building for IMU Model:
---- Building for platform: NONE
---- Building for interface: UART
-- Configuring done
-- Generating done
-- Build files have been written to: /home/user/catkin_ws/build
####
#### Running command: "make -j4 -l4" in "/home/user/catkin_ws/build"
####
Scanning dependencies of target ess_imu_driver_lib
[ 50%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/hcl_uart.c.o
[ 50%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/sensor_epsonCommon.c.o
[ 50%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/hcl_linux.c.o
[ 50%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/hcl_gpio.c.o
[ 62%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/sensor_epsonUart.c.o
[ 75%] Linking C shared library /home/user/catkin_ws/devel/lib/libess_imu_driver_lib.so
[ 75%] Built target ess_imu_driver_lib
Scanning dependencies of target ess_imu_driver_node
[ 87%] Building CXX object ess_imu_driver/CMakeFiles/ess_imu_driver_node.dir/src/epson_imu_uart_ros_node.cpp.o
[100%] Linking CXX executable /home/user/catkin_ws/devel/lib/ess_imu_driver/ess_imu_driver_node
[100%] Built target ess_imu_driver_node
```

### Example console output of launching ROS node

```
user@user:~/catkin_ws$ roslaunch ess_imu_driver normal.launch
... logging to /home/user/.ros/log/ffc82936-505a-11ef-8a91-919aef8bfaba/roslaunch-user-2855826.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://user:46575/

SUMMARY
========

PARAMETERS
 * /ess_imu_driver_node/accel_out: 1
 * /ess_imu_driver_node/atti_profile: 0
 * /ess_imu_driver_node/checksum_out: 1
 * /ess_imu_driver_node/count_out: 1
 * /ess_imu_driver_node/dout_rate: 9
 * /ess_imu_driver_node/ext_pol: 0
 * /ess_imu_driver_node/ext_sel: 1
 * /ess_imu_driver_node/filter_sel: 9
 * /ess_imu_driver_node/flag_out: 1
 * /ess_imu_driver_node/frame_id: imu_link
 * /ess_imu_driver_node/gyro_out: 1
 * /ess_imu_driver_node/qtn_out: 1
 * /ess_imu_driver_node/serial_port: /dev/ttyUSB0
 * /ess_imu_driver_node/temp_out: 1
 * /ess_imu_driver_node/time_correction: 0
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES
  /
    ess_imu_driver_node (ess_imu_driver/ess_imu_driver_node)

auto-starting new master
process[master]: started with pid [2856243]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 4a2eb51a-505d-11ef-8a91-919aef8bfaba
process[rosout-1]: started with pid [2856253]
started core service [/rosout]
process[ess_imu_driver_node-2]: started with pid [2856260]
[ INFO] [1722554708.456803065]: Initializing HCL layer...
...done.
[ INFO] [1722554708.457636356]: Initializing GPIO interface...
...done.
[ INFO] [1722554708.457663540]: Initializing UART interface...
Attempting to open port.../dev/ttyUSB0...done.
...sensorDummyWrite......done.
[ INFO] [1722554708.772857535]: Checking sensor NOT_READY status...
...done.
[ INFO] [1722554709.004028937]: Detecting sensor model...

Reading device model and serial number...
PRODUCT ID:     G370PDS0
SERIAL ID:      W0000010...done.
[ INFO] [1722554709.131947637]: Initializing Sensor...
...done.
[ INFO] [1722554709.151342038]: Epson IMU initialized.

...Sensor start.
```

## What does this ROS IMU Node publish as messages?

The Epson IMU ROS node will publish messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

- For IMU models that do not support quaternion output, the `normal.launch` file will remap IMU messages to `/imu/data_raw` and only update the fields for `angular_velocity` (gyro), and `linear_acceleration` (accel).
- For IMU models that supports and enables quaternion, the `normal.launch` file will remap IMU messages to `/imu/data` and update `angular_velocity` (gyro), `linear_acceleration` (accel), and `orientation` field using the internal extended Kalman Filter.
- Temperature sensor data from the IMU will publish on topic `/epson_imu/tempc`

### ROS Topic Message /imu/data

```
---
header:
  seq: 34983
  stamp:
    secs: 1601590579
    nsecs: 771273325
  frame_id: "imu_link"
orientation:
  x: 2.4786295434e+33
  y: 1.1713334935e+38
  z: 1.17130631507e+38
  w: 1.17130956026e+38
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.00435450254008
  y: 0.000734272529371
  z: -9.40820464166e-05
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.730921983719
  y: -1.54766368866
  z: 9.72711181641
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

```

### ROS Topic Message /imu/data_raw

```
---
header:
  seq: 10608
  stamp:
    secs: 1601575987
    nsecs: 673387357
  frame_id: "imu_link"
orientation:
  x: -0.0801454782486
  y: 0.0367396138608
  z: 0.00587898213416
  w: 0.996088504791
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: -0.00118702522013
  y: 0.000320095219649
  z: -0.00014466587163
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.727666378021
  y: -1.5646469593
  z: 9.69056034088
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

## Why am I seeing inaccurate ROS timestamps, high latencies, or slower than expected IMU data rates?

### ROS timestamps

- By default, the ROS node publishes using the ROS/Unix current timestamp at the time of processing.
- Latencies from system overhead, system loading, and buffering will cause timestamp inaccuracies between the sensor data and this ROS/Unix timestamp.
- To improve timestamp accuracy, the user should consider using the time correction function with a GNSS receiver with 1PPS output to increase clock accuracy on the ROS host system:
  - Consider setting up [chrony](https://chrony-project.org/examples.html) and [gpsd](https://gpsd.io/) with the GNSS receiver's 1PPS signal connected to constrain the host system clock drift
  - Consider connecting the GNSS receiver's 1PPS signal to the Epson IMU GPIO2/EXT with `time_correction` enabled in the `.py` launch file to improve time-stamp accuracy of the IMU sensor messages

### When using USB-UART bridges

- If your connection between the Epson IMU UART interface and the Linux host is by FTDI ICs, the `latency_timer` setting in the FTDI driver may be large i.e. typically 16 (msec).
- This may affect the UART latency and maximum IMU data rates on your host system.
- There are 3 methods listed below to reduce the impact of this latency.

#### Modifying *latency_timer* by udev mechanism

- [udev](https://wiki.debian.org/udev) is a device manager for Linux that can dynamically create and remove devices in *userspace* and run commands when new devices appear or other events
- Create a udev rule to automatically set the `latency_timer` to 1 when an FTDI USB-UART device is plugged in to a USB port.
- For example, the following text file named `99-ftdi_sio.rules` can be put in the `/etc/udev/rules.d` directory

**NOTE:** This requires root (sudo) access to create or copy file in `/etc/udev/rules.d`
**NOTE:** This is the recommended method because it is automatic when device is plugged in, but it affects ALL FTDI USB-UART devices on the system

```
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

#### Modifying *latency_timer* by sysfs mechanism

- The example below reads the `latency_timer` setting for `/dev/ttyUSB0` which returns 16msec.
- Then, it sets the `latency_timer` to 1msec, and confirms it by read back.

**NOTE: This may require root (sudo su) access on your system to execute.**

```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

#### Modifying *low_latency* flag using *setserial* utility

- The example below sets the `low_latency` flag for `/dev/ttyUSB0`.
- This will have the same effect as setting the `latency_timer` to 1msec.
- This can be confirmed by running the `setserial` command again.

```
user@user:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0

user@user:~$ setserial /dev/ttyUSB0 low_latency

user@user:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0, Flags: low_latency
```

### When using the SPI interface

- latency issues with SPI interface will largely depend on your host system's processing load and capabilities.
- If your ROS platform is running too many ROS node packages or too slow, it may not react fast enough to detect the rising edge of the IMU DRDY signal and
  process the IMU sampling data.
- Try modifying the `dout_rate` and `filter_sel` setting in the `.launch` file to the slowest setting that can meet your system requirements.
- Try monitoring the DRDY signal on the IMU with a oscilloscope to verify the stability of the IMU DRDY signal and seeing that it matches the expected `dout_rate` frequency.
- Try to set the SPI clock rate to maximum of 1MHz (1000000) in the `_node.cpp` for the `spiInit()` function call:

```
...

  ROS_INFO("Initializing SPI interface...");
  // The max SPI clock rate is 1MHz for current model Epson IMUs
  if (!spiInit(SPI_MODE3, 1000000)) {
    ROS_ERROR("Error: could not initialize SPI interface. Exiting...");
    gpioRelease();
    seRelease();
    return false;
  }

...
```

## Package Contents

The Epson IMU ROS1 driver-related sub-folders & root files are:

```
   launch/        <== various example launch files
   src/           <== source code for ROS node C++ wrapper, IMU Linux C driver, and additional README_src.md specifically for building and using the IMU Linux C driver as stand-alone (without ROS support)
   CMakeLists.txt <== cmake build script used by catkin_make
   LICENSE.txt    <== description of the applicable licenses
   package.xml    <== catkin package description
   README.md      <== general README
```

## References

1. https://index.ros.org
