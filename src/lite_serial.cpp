/*
 * lite_serial.cpp
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

#include "lite_serial.hpp"

/*
 * Constractor()
 */
lite_serial::LiteSerial::LiteSerial() {
    // Set serial descriptor
    serial_fd_ = invalid_value_;

    // Initialize termio with zero
    bzero(&tio_new_, sizeof(tio_back_));
    bzero(&tio_new_, sizeof(tio_new_));

    // True: connected, False: disconnected
    is_connected_ = false;
}

/*
 * Destructor
 */
lite_serial::LiteSerial::~LiteSerial() {
    if (serial_fd_ >= 0) {
        // Flush input & output buffer
        tcflush(serial_fd_, TCIFLUSH);
        tcflush(serial_fd_, TCOFLUSH);

        // Store configuration
        tcsetattr(serial_fd_, TCSANOW, &tio_back_);

        // Release fd
        close(serial_fd_);
        
        // Set serial descriptor
        serial_fd_ = invalid_value_;
    }
}

/*
 * Open(), binary data, raw, blocking mode
 */
bool lite_serial::LiteSerial::Open(void) {
    // Open serial port, R+W able, NOCTTY mode
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);

    // Connected or not
    if (serial_fd_ < 0) {
        is_connected_ = false;
        return false;
    }

    // Get & Save existed configuration
    tcgetattr(serial_fd_, &tio_back_);

    // Initialize with zero
    bzero(&tio_new_, sizeof(tio_new_));

    // Set flag with CS8-bits, local mode, readable, parity: none, hw/sw flow control: none, stop bit:1, canonical: none
    tio_new_.c_cflag = (CS8 | CLOCAL | CREAD);
    tio_new_.c_iflag = IGNPAR;
    tio_new_.c_oflag = 0;
    tio_new_.c_lflag = 0;

    // Set timeout value & minimum size
    tio_new_.c_cc[VTIME] = 0;
    tio_new_.c_cc[VMIN]  = rx_size_;// blocking

    // Enable new config
    tcsetattr(serial_fd_, TCSANOW, &tio_new_);

    // Flush input & output once
    tcflush(serial_fd_, TCIFLUSH);
    tcflush(serial_fd_, TCOFLUSH);

    struct serial_struct serial_setting;
    ioctl(serial_fd_, TIOCGSERIAL, &serial_setting);
    serial_setting.flags |= ASYNC_LOW_LATENCY;
    ioctl(serial_fd_, TIOCSSERIAL, &serial_setting);

    // Setup is_connected_
    is_connected_ = true;
    return is_connected_;
}

/*
 * AvailableSize(), Get available bytes number stocked @FIFO 
 */
size_t lite_serial::LiteSerial::AvailableSize(void) {
    if (is_connected_) {
        size_t rx_size = ioctl(serial_fd_, TIOCINQ, &tio_new_);
        if (rx_size < 0) { // error
            is_connected_ = false;
        }
        return rx_size;
    }
    return zero_byte_;
}

/*
 * WriteBuff(), Write binary data, non-canonical, raw mode
 */
size_t lite_serial::LiteSerial::WriteBuff(const uint8_t *buffer, const size_t size) {
    if (is_connected_) {
        if (size > 0) {
            size_t tx_size = write(serial_fd_, buffer, size);
            if (tx_size < 0) { // error
                is_connected_ = false;
            }
            return tx_size;
        }
    }
    return zero_byte_;
}

/*
 * ReadBuff(), Read binary data, non-canonical, raw, blocking mode
 */
size_t lite_serial::LiteSerial::ReadBuff(uint8_t * buffer, const size_t size) {
    if (is_connected_) {
        if (size > 0) {
            size_t rx_size = read(serial_fd_, buffer, size);

            if (rx_size < 0) { // error
                is_connected_ = false;
            }
            return rx_size;
        }
   }
   return zero_byte_;
}

/*
 * GetIsConnected(), Getter of is_connected_
 */
bool lite_serial::LiteSerial::GetIsConnected(void) {
    return is_connected_;
}

/*
 * SetIsConnected(), Setter of is_connected_
 */
void lite_serial::LiteSerial::SetIsConnected(bool is_connected) {
    is_connected_ = is_connected;
}

/*
 * GetSerialFd(), Getter of serial_fd_
 */
int32_t lite_serial::LiteSerial::GetSerialFd(void) {
    return serial_fd_;
}

/*
 * GetTermio(), Getter of termio_
 */
struct termios lite_serial::LiteSerial::GetTermio(void) {
    return tio_back_;
}

/*
 * FlushBuffer(), Flush input & output buffer
 */
void lite_serial::LiteSerial::FlushBuffer(void) {
    // Flush input & output buffer
    tcflush(serial_fd_, TCIFLUSH);
    tcflush(serial_fd_, TCOFLUSH);
}

/*
 * Close(), Close serial port, flush buffer, store termio
*/
void lite_serial::LiteSerial::Close() {
    if (serial_fd_ >= 0) {
        // Flush buffer
        LiteSerial::FlushBuffer();

        // Store configuration
        tcsetattr(serial_fd_, TCSANOW, &tio_back_);

        // Release fd
        close(serial_fd_);
    }
} // lite_serial.hpp