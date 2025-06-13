// Copyright 2025 TESOLLO
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the TESOLLO nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once
#include <array>
#include <boost/asio.hpp>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

struct DeltoReceivedData {
  std::vector<double> joint;        // radian
  std::vector<double> current;      // A
  std::vector<double> temperature;  // Celsius
  std::vector<double> velocity;     // rad/s
};

using boost::asio::ip::tcp;
namespace asio = boost::asio;

namespace DeltoTCP {

class Communication {
 public:
  Communication(const std::string& ip, int port, int16_t model,
                               bool fingertip_sensor, bool io);
  ~Communication();

  void Connect();
  void Disconnect();
  DeltoReceivedData GetData();
  void SendDuty(std::vector<int>& duty);
  bool ReadFullPacket(boost::asio::ip::tcp::socket& socket,
                      std::vector<uint8_t>& buffer);

 private:
  std::string ip_;
  int port_;
  int16_t model_;
  bool fingertip_sensor_;
  bool io_;
  asio::io_context io_context_;
  tcp::socket socket_;

      // ID + PosL + PosH + CurL + CurH + TempL + TempH + Vel +
      // other(fingertip_sensor, io)
  const int motor_count_;
  const int byte_per_motor_;
  const int total_duty_packet_size_;  // 전체 패킷 크기
  const int16_t expected_response_length_;  // 예상되는 패킷 길이 

  static constexpr uint8_t GET_DATA_CMD = 0x01;
  static constexpr uint8_t SET_DUTY_CMD = 0x05;

  static constexpr std::size_t HEADER_SIZE = 3;  // Length(2) + CMD(1)
  static constexpr double POSITION_SCALE = (M_PI / 1800.0);
  static constexpr double CURRENT_SCALE = 1000.0;
  static constexpr double VELOCITY_SCALE = (M_PI / 1800.0);
  
  int GetBytePerMotor (bool fingertip_sensor, bool io);
  int GetMotorCount(uint16_t model);
  // helper functions
  int16_t CombineMsg(uint8_t data1, uint8_t data2);
  int8_t convertByte(uint8_t data);
};
}  // namespace DeltoTCP
