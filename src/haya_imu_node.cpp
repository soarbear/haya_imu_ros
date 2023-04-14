/*
 * haya_imu_node.cpp
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

#include "haya_imu_node.hpp"

/*
 * HayaImuNode() HayaImuNode class constractor
 */
HayaImuNode::HayaImuNode() : imu_topic_("/imu_data"),
                             imu_frame_("imu_link"),
                             imu_mode_(ODR_500Hz_MODE) {
    ros::NodeHandle n("~");
    ros::NodeHandle nh;
    int32_t imu_mode;

    // Get parameters from config file, otherwise takes default value
    n.param<std::string>("serial_port_name", port_name_, "/dev/ttyACM_haya");
    n.param<std::string>("imu_topic", imu_topic_, "/imu_data");
    n.param<std::string>("imu_frame", imu_frame_, "imu_link");
    n.param<int32_t>("imu_mode", imu_mode, ODR_500Hz_MODE);
    imu_mode_ = static_cast <int16_t> (imu_mode);

    // Normal output mode or Calibration mode
    publish_imu_ = nh.advertise<haya_imu_ros::ImuData>(imu_topic_, 1);

    // Covariance matrix
    for (int32_t i = 0; i < sizeof(angular_velocity_covariance_) / sizeof(double); i++) {
        imu_msg_.angular_velocity_covariance[i] = angular_velocity_covariance_[i];
        imu_msg_.linear_acceleration_covariance[i] = linear_acceleration_covariance_[i];
        imu_msg_.magnetic_field_covariance[i] = magnetic_field_covariance_[i];
    }
    
    // Create CRC table
    CreateCrcTable();

    // Try connecting serial port
     while (ros::ok()) {
        if (!LiteSerial::Open()) { // Disconnected
            ROS_INFO_ONCE("[Open] Try opening serial: %s, which can be changed in params.yaml", port_name_.c_str());
            ros::Duration(1.0).sleep(); // Sleep for 1.0s, try modifying to shorter or longer value according to needs
        }
        else { // Connected
            return;
        }
     }
}

/*
 * ~HayaImuNode() HayaImuNode class constractor
*/
HayaImuNode::~HayaImuNode() {
}

/*
 * SendParams() Send parameters through serial port
 */
void HayaImuNode::SendParams() {
    int32_t response[2];

    // Check parameter imu_mode_ once
    if ((imu_mode_ < CALIBRATION_MODE) || (imu_mode_ > ODR_1KHZ_MODE)) { // Exception
        ROS_WARN("[Para] Parameter(imu_mode in params.yaml) be out of range, imu works in ODR_500Hz_MODE");
        imu_mode_ = ODR_500Hz_MODE;
    }
    
    // Send parameter to imu
    size_t len = LiteSerial::WriteBuff(reinterpret_cast <uint8_t*>(&imu_mode_), sizeof(imu_mode_));

    // Wait for response from imu
    while (ros::ok()) {
        if (LiteSerial::ReadBuff(reinterpret_cast <uint8_t*>(response), sizeof(response)) < sizeof(response)) {
            ROS_ERROR_ONCE("[Para] Failed to receive response of parameter setup, check USB connection & parameters in params.yaml");
            continue;
        }
        else {
            //ROS_INFO("[Para] Response of imu parmas setup received: 0x%08x,  0x%08x", response[0], response[1]);
            break;
        }
    }

    // Check the hardware version of haya_imu once
    if ((((response[0] >> 8) & 0xff) != 0x03)) { // Valid version range: v3.1.0 - v3.f.f(max)
        ROS_ERROR("[Para] Before performing a launch file, turn off then on power, or reset imu once. Press Ctrl+c to try again");
        while (ros::ok()); // Press Ctrl+c to try again
    }

    // Release information on device to ROS
    switch (imu_mode_) {
        case DEMONSTRATION_MODE:
            ROS_INFO("[Para] Serial: %s opened, Init-Bias: %s, Mode: Demo(250Hz), Firmware: v%d.%d.%d, Product_SN: %08x",
                    port_name_.c_str(), is_calibrated_[(response[0] >>16 ) &0x1].c_str(), (response[0] >> 8) & 0xf,
                    (response[0] >>4 ) & 0xf, response[0]&0xf, response[1]);
            break;
        case CALIBRATION_MODE:
            ROS_INFO("[Para] Serial: %s opened , Init-Bias: %s, Mode: Calibration(2Hz), Firmware: v%d.%d.%d, Product_SN: %08x",
                    port_name_.c_str(), is_calibrated_[(response[0] >>16 ) &0x1].c_str(), (response[0] >> 8) & 0xf,
                    (response[0] >> 4) & 0xf, response[0] & 0xf, response[1]);
            break;
        default: // Normal output mode
            ROS_INFO("[Para] Serial: %s opened, Init-Bias: %s, Mode: Normal Output(%dHz), Firmware: v%d.%d.%d, Product_SN: %08x",
                    port_name_.c_str(), is_calibrated_[(response[0]>>16)&0x1].c_str(), 
                    imu_mode_, (response[0] >> 8) & 0xf, (response[0] >> 4) & 0xf, response[0] & 0xf, response[1]);
    }
}

/*
 * ReadImuData() Read Imu Data
 */
void HayaImuNode::ReadImuData() {
    // Iteration to read imu data
    while (ros::ok()) {
        // Read imu_data_ buffer from serial port
        if (LiteSerial::ReadBuff(reinterpret_cast <uint8_t*>(&imu_data_), sizeof(imu_data_)) < sizeof(imu_data_)) { // Disconnected
            ROS_ERROR_ONCE("[Read] Serial disconnected, press Ctrl+c to stop the node, then reset haya_imu, then roslaunch again");
            continue;
        }
        else { // Buffer imu_data_ read ok
            // CRC check
            if (imu_data_.crc16 == CalCrc(reinterpret_cast <uint8_t*>(&imu_data_), sizeof(imu_data_) - sizeof(uint16_t))) {
                PublishImuMsg();
             }
            else { // CRC error
                ROS_INFO("[Read] CRC NG once. If wrong data occurs ofen, check environment & parameters in params.yaml");
            }
        }
    }
}

/*
  PublishImuData() Publish topic
 */
void HayaImuNode::PublishImuMsg() {
    // Prepare published data and publish
    if (imu_mode_ == DEMONSTRATION_MODE) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion tf_quat;
        geometry_msgs::Quaternion geometry_quat;

        // Prepare tf of fusion_6axis
        geometry_quat.w = imu_data_.quaternion[0][0];
        geometry_quat.x = imu_data_.quaternion[0][1];
        geometry_quat.y = imu_data_.quaternion[0][2];
        geometry_quat.z = imu_data_.quaternion[0][3];
        tf::quaternionMsgToTF(geometry_quat, tf_quat);
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation(tf_quat);
        ros::Time now = ros::Time::now();

        // Broadcast tf of fusion_6axis
        br.sendTransform(tf::StampedTransform(transform, now, "world", "fusion_6axis"));

        // Prepare tf of fusion_9axis
        geometry_quat.w = imu_data_.quaternion[1][0];
        geometry_quat.x = imu_data_.quaternion[1][1];
        geometry_quat.y = imu_data_.quaternion[1][2];
        geometry_quat.z = imu_data_.quaternion[1][3];
        tf::quaternionMsgToTF(geometry_quat, tf_quat);
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation(tf_quat);

        // Broadcast tf of fusion_9axis
        br.sendTransform(tf::StampedTransform(transform, now, "world", "fusion_9axis"));

        // Release information on publishing fusion data once
        ROS_INFO_ONCE("[Tfbr] Tf broadcaster fusion_6axis & fusion_9axis @ %dHz", imu_mode_);
    }
    else { // Normal output mode or Calibration mode
        // Message header
        imu_msg_.header.stamp = ros::Time::now();
        imu_msg_.header.frame_id = imu_frame_;

        // Angular velocity
        imu_msg_.angular_velocity.x = imu_data_.angular_velocity[0];
        imu_msg_.angular_velocity.y = imu_data_.angular_velocity[1];
        imu_msg_.angular_velocity.z = imu_data_.angular_velocity[2];

        // Linear acceleration
        imu_msg_.linear_acceleration.x = imu_data_.linear_acceleration[0];
        imu_msg_.linear_acceleration.y = imu_data_.linear_acceleration[1];
        imu_msg_.linear_acceleration.z = imu_data_.linear_acceleration[2];

        // Magnetic field
        imu_msg_.magnetic_field.x = imu_data_.magnetic_field[0];
        imu_msg_.magnetic_field.y = imu_data_.magnetic_field[1];
        imu_msg_.magnetic_field.z = imu_data_.magnetic_field[2];

        // 6-axis fusion quaternion
        imu_msg_.orientation[0].w = imu_data_.quaternion[0][0];
        imu_msg_.orientation[0].x = imu_data_.quaternion[0][1];
        imu_msg_.orientation[0].y = imu_data_.quaternion[0][2];
        imu_msg_.orientation[0].z = imu_data_.quaternion[0][3];

        // 9-axis fusion quaternion
        imu_msg_.orientation[1].w = imu_data_.quaternion[1][0];
        imu_msg_.orientation[1].x = imu_data_.quaternion[1][1];
        imu_msg_.orientation[1].y = imu_data_.quaternion[1][2];
        imu_msg_.orientation[1].z = imu_data_.quaternion[1][3];

        // Units accuracy index, 0x00(worst) -> 0x3f(best)
        imu_msg_.calibration_index = imu_data_.calibration_index;

        // 6-axis imu temperature
        imu_msg_.temperature_imu = imu_data_.temperature_imu;

        // Get Euler angle, rotation sequence is Yaw -> Pitch -> Roll
        GetEulerYPR(&imu_msg_);

        // Publish imu topic
        publish_imu_.publish(imu_msg_);

        // Release information om publishing fusion data once
        ROS_INFO_ONCE("[Publ] Publishing Topic: %s (haya_imu_ros::ImuData) @ %dHz", imu_topic_.c_str(), imu_mode_);
    }
}

/*
 * GetEulerYPR() Get Euler angle, refer to manual for more information
 */
void HayaImuNode::GetEulerYPR(haya_imu_ros::ImuData *p_msg) {
    geometry_msgs::Quaternion geometry_quat;
    geometry_quat.x = p_msg->orientation[0].x;
    geometry_quat.y = p_msg->orientation[0].y;
    geometry_quat.z = p_msg->orientation[0].z;
    geometry_quat.w = p_msg->orientation[0].w;
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(geometry_quat, tf_quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    p_msg->euler_ypr.z = yaw;
    p_msg->euler_ypr.y = pitch;
    p_msg->euler_ypr.x = roll;
}

/*
 * Crc16Table()  Calculate CRC : CCITT16 Table
 */
void HayaImuNode::CreateCrcTable(void) { 
    int32_t i, j;
    uint16_t crc;
    for (i = 0; i < 256; i++) {
        crc = (i << 8);
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = polynom_ ^ (crc << 1);
            }
            else {
                crc <<= 1;
            }
        }
        crc_table_[i] = crc;
    }
}

/*
 * Crc16Table()  Calculate CRC: CCITT16
 */
uint16_t HayaImuNode::CalCrc(uint8_t *data, size_t len) {
    uint32_t crc = 0xffff, final = 0x0000;
    uint32_t temp;

    for (uint32_t i = 0; i < len; ++i) {
        temp = (*data++ ^ (crc >> 8)) & 0xff;
        crc = crc_table_[temp] ^ (crc << 8);
    }

    return static_cast<uint16_t>(crc ^ final);
}

// Serial port file descriptor
int32_t serial_fd = -1;

// Struct for config restoration
struct termios tio_back;

/*
 * sigint_handler() Ctrl+c handler
*/
void sigint_handler(int sig) {
    if (serial_fd >= 0) {
        // Flush input & output buffer
        tcflush(serial_fd, TCIFLUSH);
        tcflush(serial_fd, TCOFLUSH);

        // Store configuration
        tcsetattr(serial_fd, TCSANOW, &tio_back);

        // Release fd
        close(serial_fd);
    }
    ros::shutdown();
}

/*
 * main() for haya_imu_node
 */
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "haya_imu_node", ros::init_options::NoSigintHandler);

    // Create an instance
    HayaImuNode imu_node;
    serial_fd = imu_node.GetSerialFd();
    tio_back = imu_node.GetTermio();

    // Set ctrl+c handler
    signal(SIGINT, sigint_handler);

    // Send parameters
    imu_node.SendParams();

    // Receive imu data
    imu_node.ReadImuData();

    // Shut down
    imu_node.Close();
    ROS_INFO("[Close] haya_imu_node done\n");
    return 0;
} // haya_imu_node.hpp
