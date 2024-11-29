//==============================================================================
//
// 	epson_imu_uart_driver_node.cpp
//     - ROS node for Epson IMU sensor evaluation
//     - This program initializes the Epson IMU and publishes ROS messages in
//       ROS topic /epson_imu as convention per [REP 145]
//       (http://www.ros.org/reps/rep-0145.html).
//     - If the IMU model supports quaternion output
//       then sensor messages are published topic /epson_imu/data with
//       angular_velocity, linear_acceleration, orientation fields updating
//     - If the IMU model does not support quaternion output
//       then sensor messages are published topic /epson_imu/data with only
//       angular_velocity, and linear_acceleration fields updating
//
//  [This software is BSD-3
//  licensed.](http://opensource.org/licenses/BSD-3-Clause)
//
//  Original Code Development:
//  Copyright (c) 2019, Carnegie Mellon University. All rights reserved.
//
//  Additional Code contributed:
//  Copyright (c) 2019, 2024, Seiko Epson Corp. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//  THE POSSIBILITY OF SUCH DAMAGE.
//
//==============================================================================

#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <string>
#include <termios.h>

#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_uart.h"
#include "sensor_epsonUart.h"
#include "sensor_epsonCommon.h"

using namespace std;

string serial_port;

//=========================================================================
//------------------------ IMU Initialization -----------------------------
//=========================================================================

bool init_imu(struct EpsonProperties* ptr_epson_sensor,
              struct EpsonOptions* ptr_epson_options) {
  char prod_id_[9];  // String to store Device Product ID
  char ser_id_[9];   // String to store Device Serial ID

  ROS_INFO("Initializing HCL layer...");
  if (!seInit()) {
    ROS_ERROR(
      "Error: could not initialize the Seiko Epson HCL layer. Exiting...");
    return false;
  }
  std::cout << "...done." << std::endl;

  ROS_INFO("Initializing GPIO interface...");
  if (!gpioInit()) {
    ROS_ERROR("Error: could not initialize the GPIO layer. Exiting...");
    seRelease();
    return false;
  }
  std::cout << "...done." << std::endl;

  ROS_INFO("Initializing UART interface...");
  // The baudrate value should be set the the same setting as currently
  // flashed value in the IMU UART_CTRL BAUD_RATE register
  if (!uartInit(serial_port.c_str(), BAUD_460800)) {
    ROS_ERROR("Error: could not initialize UART interface. Exiting...");
    gpioRelease();
    seRelease();
    return false;
  }
  std::cout << "...done." << std::endl;

  // This is a workaround for NVIDIA Jetson TK1.
  // When the serial port is first opened, some random
  // characters are sent on first serial messages. This
  // attempts to workaround this issue.
  sensorDummyWrite();
  std::cout << "...done." << std::endl;

  ROS_INFO("Checking sensor NOT_READY status...");
  if (!sensorPowerOn()) {
    ROS_ERROR("Error: failed to power on Sensor. Exiting...");
    uartRelease();
    gpioRelease();
    seRelease();
    return false;
  }
  std::cout << "...done." << std::endl;

  ROS_INFO("Detecting sensor model...");
  if (!sensorGetDeviceModel(ptr_epson_sensor, prod_id_, ser_id_)) {
    ROS_ERROR("Error: failed to detect sensor model. Exiting...");
    return false;
  }
  std::cout << "...done." << std::endl;

  ROS_INFO("Initializing Sensor...");
  if (!sensorInitOptions(ptr_epson_sensor, ptr_epson_options)) {
    ROS_ERROR("Error: could not initialize Epson Sensor. Exiting...");
    uartRelease();
    gpioRelease();
    seRelease();
    return false;
  }
  std::cout << "...done." << std::endl;

  ROS_INFO("Epson IMU initialized.\n");
  return true;
}

//=========================================================================
// Timestamp Correction
//
// Time correction makes use of the Epson IMU External Reset Counter function.
// This assumes that the ROS time is accurately sync'ed to GNSS (approx.
// within 100s of microsecs) and the GNSS 1PPS signal is sent to
// Epson IMU's GPIO2_EXT pin. The system latency for calling
// rclcpp::Clock().now() is assumed to be negligible. Otherwise the timestamp
// correction may not be reliable. The get_stamp() method attempts to return
// a timestamp based on the IMU reset count value to exclude time delays
// caused by latencies in the link between the host system and the IMU.
//=========================================================================

class TimeCorrection {
 private:
  const int64_t ONE_SEC_NSEC = 1000000000;
  const int64_t HALF_SEC_NSEC = 500000000;
  // For Gen2 IMUs freq = 46875Hz, max_count = 65535/46875 * 1e9
  const int64_t GEN2_MAX = 1398080000;
  // For Gen3 IMUs freq = 62500Hz, max_count = 65535/62500 * 1e9
  const int64_t GEN3_MAX = 1048560000;
  int64_t max_count;
  int64_t almost_rollover;
  int64_t count_corrected;
  int64_t count_corrected_old;
  int64_t count_old;
  int64_t count_diff;
  int32_t time_sec_current;
  int32_t time_sec_old;
  int64_t time_nsec_current;
  bool rollover;
  bool flag_imu_lead;
  bool is_gen2_imu;

 public:
  TimeCorrection();
  void set_imu(int);
  ros::Time get_stamp(int);
};

// Constructor
TimeCorrection::TimeCorrection() {
  max_count = GEN3_MAX;
  almost_rollover = max_count;
  count_corrected = 0;
  count_old = 0;
  count_diff = 0;
  time_sec_current = 0;
  time_sec_old = 0;
  time_nsec_current = 0;
  count_corrected_old = 0;
  rollover = false;
  flag_imu_lead = false;
  is_gen2_imu = false;
}

//=========================================================================
// TimeCorrection::set_imu
//
// Sets the count thresholds based on external counter reset frequencies
// which may vary depending on the Epson IMU model.
//=========================================================================

void TimeCorrection::set_imu(int epson_model) {
  // max_count depends on IMU model's reset counter freq
  is_gen2_imu = ((epson_model == G320PDG0) || (epson_model == G320PDGN) ||
                 (epson_model == G354PDH0) || (epson_model == G364PDCA) ||
                 (epson_model == G364PDC0));

  max_count = (is_gen2_imu) ? GEN2_MAX : GEN3_MAX;
}

//=========================================================================
// TimeCorrection::get_stamp
//
// Returns the timestamp based on time offset from most recent 1PPS signal.
// Epson IMU has a free-running up-counter that resets on active 1PPS signal.
// Counter value is embedded in the sensor data at the time of sampling.
// Time stamp is corrected based on reset counter retrieved from embedded
// sensor data.
//=========================================================================
ros::Time TimeCorrection::get_stamp(int count) {
  time_sec_current = ros::Time::now().toSec();
  time_nsec_current = ros::Time::now().nsec;

  // almost_rollover is arbitrarily set at ~96% of max_count
  almost_rollover = max_count * 0.96;

  count_diff = count - count_old;
  if (count > almost_rollover) {
    rollover = true;
  } else if (count_diff < 0) {
    if (rollover) {
      count_diff = count + (max_count - count_old);
      ROS_WARN(
        "Warning: time_correction enabled but IMU reset counter "
        "rollover detected. If 1PPS not connected to IMU GPIO2/EXT "
        "pin, disable time_correction.");
    } else {
      count_diff = count;
      count_corrected = 0;
    }
    rollover = false;
  }
  count_corrected = (count_corrected + count_diff) % ONE_SEC_NSEC;
  if ((time_sec_current != time_sec_old) && (count_corrected > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current - 1;
  } else if (((count_corrected - count_corrected_old) < 0) &&
             (time_nsec_current > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current + 1;
    flag_imu_lead = true;
  } else if ((flag_imu_lead) && (time_nsec_current > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current + 1;
  } else {
    flag_imu_lead = false;
  }
  ros::Time time;
  time.nsec = count_corrected;
  time.sec = time_sec_current;
  time_sec_old = time_sec_current;
  count_old = count;
  count_corrected_old = count_corrected;

  return time;
}

//=========================================================================
//------------------------------ Main -------------------------------------
//=========================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "ess_imu_driver_node");

  // Force flush of the stdout buffer, which ensures a sync of all console
  // output even from a launch file.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  ros::NodeHandle nh;
  ros::NodeHandle np("~");

  std::string frame_id = "imu_link";
  std::string imu_topic = "epson_imu";
  std::string temperature_topic = "epson_tempc";

  np.param<string>("serial_port", serial_port, "/dev/ttyUSB0");

  // IMU properties
  struct EpsonProperties epson_sensor;

  // IMU configuration settings
  struct EpsonOptions epson_options = {};

  // Buffer for scaled values of IMU read burst
  struct EpsonData epson_data = {};

  // Time correction object
  TimeCorrection tc;

  sensor_msgs::Temperature tempc_msg;
  sensor_msgs::Imu imu_msg;

  int time_correction = false;

  // Recommend changing these parameters by .launch file instead of
  // modifying source code below directly
  np.param("ext_sel", epson_options.ext_sel, 1);  // external reset counter
  np.param("ext_pol", epson_options.ext_pol, 0);
  np.param("drdy_on", epson_options.drdy_on, 1);
  np.param("drdy_pol", epson_options.drdy_pol, 1);

  np.param("dout_rate", epson_options.dout_rate, CMD_RATE250);
  np.param("filter_sel", epson_options.filter_sel, CMD_FLTAP32);

  np.param("flag_out", epson_options.flag_out, 1);
  np.param("temp_out", epson_options.temp_out, 1);
  np.param("gyro_out", epson_options.gyro_out, 1);
  np.param("accel_out", epson_options.accel_out, 1);
  np.param("qtn_out", epson_options.qtn_out, 1);
  np.param("count_out", epson_options.count_out, 1);
  np.param("checksum_out", epson_options.checksum_out, 1);

  np.param("temp_bit", epson_options.temp_bit, 1);
  np.param("gyro_bit", epson_options.gyro_bit, 1);
  np.param("accel_bit", epson_options.accel_bit, 1);
  np.param("qtn_bit", epson_options.qtn_bit, 1);

  np.param("atti_mode", epson_options.atti_mode, 1);
  np.param("atti_profile", epson_options.atti_profile, 0);

  np.param("time_correction", time_correction, 0);

  if (!init_imu(&epson_sensor, &epson_options)) return -1;

  // if quaternion output is enabled set topic to /epson_imu/data
  // Otherwise set topic to /epson_imu/data_raw
  imu_topic = (static_cast<bool>(epson_options.qtn_out) == false)
                ? "/epson_imu/data_raw"
                : "/epson_imu/data";

  // Initialize time correction thresholds if enabled
  if (time_correction) {
    tc.set_imu(epson_sensor.model);
  }

  ros::Publisher tempc_pub =
    nh.advertise<sensor_msgs::Temperature>(temperature_topic, 1);

  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0;
    imu_msg.angular_velocity_covariance[i] = 0;
    imu_msg.linear_acceleration_covariance[i] = 0;
  }
  imu_msg.orientation_covariance[0] = -1;

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);

  sensorStart();

  while (ros::ok()) {
    if (sensorDataReadBurstNOptions(&epson_sensor, &epson_options,
                                    &epson_data)) {
      imu_msg.header.frame_id = frame_id;
      if (!time_correction)
        imu_msg.header.stamp = ros::Time::now();
      else
        imu_msg.header.stamp = tc.get_stamp(epson_data.count);
      imu_msg.angular_velocity.x = epson_data.gyro_x;
      imu_msg.angular_velocity.y = epson_data.gyro_y;
      imu_msg.angular_velocity.z = epson_data.gyro_z;
      imu_msg.linear_acceleration.x = epson_data.accel_x;
      imu_msg.linear_acceleration.y = epson_data.accel_y;
      imu_msg.linear_acceleration.z = epson_data.accel_z;

      // Publish quaternion orientation
      imu_msg.orientation.x = epson_data.qtn1;
      imu_msg.orientation.y = epson_data.qtn2;
      imu_msg.orientation.z = epson_data.qtn3;
      imu_msg.orientation.w = epson_data.qtn0;
      imu_pub.publish(imu_msg);

      // Publish temperature
      tempc_msg.header = imu_msg.header;
      tempc_msg.temperature = epson_data.temperature;
      tempc_pub.publish(tempc_msg);

    } else {
      ROS_WARN(
        "Warning: Checksum error or incorrect delimiter bytes in imu_msg "
        "detected");
    }
  }
  sensorStop();
  seDelayMS(1000);
  uartRelease();
  gpioRelease();
  seRelease();

  return 0;
}
