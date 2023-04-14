/*
 * lite_serial.hpp
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

#include <cstdint>
#include <cstring>
#include <string>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

namespace lite_serial {
    class LiteSerial {
        protected:
            // Serial port file descriptor
            int32_t serial_fd_;

            // True: connected, False: disconnected
            bool is_connected_;

            // Return code
            static constexpr size_t zero_byte_ = 0;
            static constexpr int32_t invalid_value_ = -2;
            static constexpr int32_t rx_size_ = 128;
            static constexpr int32_t tx_size_ = 128;

        private:
            // Struct for termio configuration
            struct termios tio_back_;
            struct termios tio_new_;

        public:
            // Serial port
            std::string port_name_;

            // Constructor
            LiteSerial();

            // Desstructor
            ~LiteSerial();

            /*
             * Open(), Open Serial port, binary data, non-canonical, raw, blocking mode
             */
            bool Open(void);

            /*
             * AvailableSize(), Get available bytes number stocked @FIFO 
             */
            size_t AvailableSize(void);

            /*
             * WiteBuff(), binary data, non-canonical, raw mode
             */
            size_t WriteBuff(const uint8_t *buffer, const size_t len);

            /*
             * ReadBuff() , binary data, non-canonical, raw, blocking mode
             */
            size_t ReadBuff(uint8_t * buffer, const size_t size);

            /*
             * GetIsConnectedr(), Getter of is_connected_
             */
            bool GetIsConnected(void);

            /*
             * SetIsConnectedr(), Setter of is_connected_
             */
            void SetIsConnected(bool is_connected);

            /*
             * GetSerialFd(), Getter of serial_fd_
             */
            int32_t GetSerialFd(void);

            /*
             * GetSerialFd(), Getter of termio
             */
            struct termios GetTermio(void);

            /*
             * GetSerialFd(), Setter of termio
             */
            void SetTermio(termios tio);

            /*
             * FlushBuffer(), Flush input & output buffer
             */
            void FlushBuffer(void);

            /*
             * Close(), Close serial port, flush buffer, store termio
             */
            void Close(void);
    }; // class LiteSerial
} // namespace lite_serial
// lite_serial.hpp