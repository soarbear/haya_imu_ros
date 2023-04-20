/*
 * haya_imu_node.hpp
 *
 * License: BSD-3-Clause
 *
 * Copyright (c) 2012-2022 Shoun Corporation <research.robosho@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of RT Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "lite_serial.hpp"
#include <signal.h>
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "haya_imu_ros/ImuData.h"

class HayaImuNode : public lite_serial::LiteSerial {
	public:
		// Constructor
		HayaImuNode();

		// Destructor
		~HayaImuNode();

		/*
		 * Send parameters to haya_imu
		 */
		void SendParams(void);

		/*
		 * Read imu data(fusioned & calibrated by imu device) from serial port
		 */
		void ReadImuData(void);

		/*
		 * Publish imu message(ROS)
		 */	
		void PublishImuMsg(void);

	private:
		// ROS topic & frame id
		std::string imu_topic_;
		std::string imu_frame_;

		// Haya_imu operation mode(refer to config/params.yaml)
		int16_t imu_mode_;

		// Imu meassage for Normal output mode and Calibration mode
		haya_imu_ros::ImuData imu_msg_;

		// Haya_imu service mode enum
		typedef enum {
			ODR_1KHZ_MODE		= 1000,	// (Output Data Mode,	1000Hz Output Data Rate)
			ODR_500Hz_MODE		= 500,	// (Output Data Mode,	500Hz Output Data Rate, default)
			DEMONSTRATION_MODE	= 250,	// (Demonstration Mode,	250Hz Output Data Rate)
			ODR_100Hz_MODE		= 100,	// (Output Data Mode, 	25Hz Output Data Rate)
			CALIBRATION_MODE	= 10,	// (Calibration Mode,	10Hz Output Data Rate)
		} ServiceMode;

		// Data read from haya_imu through serial port
		struct HayaImuData {
			int64_t  time_stamp_imu; // time_stamp of imu(accelerometer & gyroscope)
			int64_t  time_stamp_magnetometer; // time_stamp of magetometer, ignore magnetic_field while = -1
			float    angular_velocity[3]; // dps
			float    linear_acceleration[3]; // m/s^2
			float    magnetic_field[3]; // uT
			float    bias_gyroscope[3]; // dps
			float    bias_acceleration[3]; // g
			float    bias_magnetometor[3]; // uT
			float    quaternion[2][4]; // 6axis & 9axis fusion data
			float    reservation; // reaservation 
			int8_t   temperature_imu; // celsius deg
			uint8_t  calibration_index; // combined calibration index, range: [0, 63] 
			uint16_t crc16; // CRC16 CTITT16
		} imu_data_;

		// Angular velocity covariance matrix 3x3
		const double angular_velocity_covariance_[9] = { 
			0.028*0.028,			0.0,					0.0,
			0.0,					0.028*0.028,			0.0,
			0.0,					0.0,					0.028*0.028};

		// Linear acceleration covariance matrix 3x3       
		const double linear_acceleration_covariance_[9] = {
			0.65*0.001*9.8*0.65*0.001*9.8,		0.0,								0.0,
			0.0,								0.65*0.001*9.8*0.65*0.001*9.8,		0.0,
			0.0,								0.0,								0.70*0.001*9.8*0.70*0.001*9.8};

		// Magnetic field covariance matrix 3x3
		const double magnetic_field_covariance_[9] = {
			0.12*0.12,				0.0,					0.0,
			0.0,					0.12*0.12,				0.0,
			0.0,					0.0,					0.12*0.12};

		const std::string is_calibrated_[2] = {"Uncalibrated", "Calibrated"};

		// CRC Polynomal
		const uint16_t polynom_ =  0x1021;

		// CRC = CCITT16 table 
		uint16_t crc_table_[256];

		// Topic to publish
		ros::Publisher publish_imu_;

		/*
		 * Get Euler angle Yaw-Pitech-Roll
		 */
		void GetEulerYPR(haya_imu_ros::ImuData *p_msg);

		/*
		 * Create CRC table, CCITT16
		 */
		void CreateCrcTable();

		/*
		 * Calculate CRC value, CCITT16
		 */
		uint16_t CalCrc(uint8_t *data, size_t len);
}; // class HayaImuNode
// haya_imu_node.hpp
