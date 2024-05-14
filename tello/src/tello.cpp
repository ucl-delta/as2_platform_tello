// Copyright 2024 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
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

/**
 * @file tello.cpp
 *
 * Implements socket communication with the tello drone.
 *
 * @authors Daniel Fernández Sánchez
 */

#include "tello.hpp"
#include <sys/socket.h>

static std::vector<std::string> split(const std::string & target, char c)
{
  std::string temp;
  std::stringstream stringstream{target};
  std::vector<std::string> result;

  while (std::getline(stringstream, temp, c)) {
    result.emplace_back(temp);
  }

  return result;
}

Tello::Tello(const std::string& command_ip, const int command_port, 
             const std::string& state_ip, const int state_port)
{
  commandSender_ = std::make_unique<SocketUdp>(command_ip, command_port);
  stateRecv_ = std::make_unique<SocketUdp>(state_ip, state_port);
}

Tello::~Tello()
{
  connected_ = false;
  if (stateThd_.joinable()) {
    stateThd_.join();
  }
  if (videoThd_.joinable()) {
    videoThd_.join();
  }
  std::cout << "Clean exit." << std::endl;
}

bool Tello::connect(const double state_update_rate)
{
  for (int i = 0; i < 3; ++i) {
    if (sendCommand("command")) {
      connected_ = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (!connected_) {
    std::cout << "Error: Connecting to Tello" << std::endl;
    return false;
  }
  std::cout << "Tello connection established!" << std::endl;
  stateRecv_->bindServer();
  // update();

  state_update_interval_ = 1.0 / state_update_rate;
  stateThd_ = std::thread(&Tello::threadStateFnc, this);
  streamVideoStart();
  return connected_;
}

bool Tello::sendCommand(const std::string & command, bool wait, std::string * response)
{
  std::string msgs_back = "";
  if (!wait) {
    commandSender_->sending(command);
    msgs_back = commandSender_->receiving(MSG_DONTWAIT);
    return true;
  }

  commandSender_->receiving(MSG_DONTWAIT);
  commandSender_->sending(command);
  msgs_back = commandSender_->receiving(MSG_WAITFORONE);
  if (response != nullptr) {
    *response = msgs_back;
  }
  return strcmp(msgs_back.c_str(), "ok") == 0;
}

void Tello::threadStateFnc()
{
  while (connected_) {
    if (getState()) {update();}
    std::this_thread::sleep_for(std::chrono::duration<double>(state_update_interval_));
  }
}

void Tello::streamVideoStart(const double camera_update_rate, const std::string & stream_ip, const int stream_port)
{
  camera_stream_url_ = "udp://" + stream_ip + ":" + std::to_string(stream_port);
  camera_update_interval_ = 1.0 / camera_update_rate;
  videoThd_ = std::thread(&Tello::streamVideo, this);
}


void Tello::streamVideoStop()
{
  bool response = sendCommand("streamoff");

  if (response) {
    streaming_ = false;
  }
}

void Tello::streamVideo()
{
  bool response = sendCommand("streamon");

  if (response) {
    cv::VideoCapture capture{camera_stream_url_.c_str(), cv::CAP_FFMPEG};
    cv::Mat frame;

    streaming_ = true;

    while (connected_ && streaming_) {
      capture >> frame;
      if (!frame.empty()) {
        frame_ = frame;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(camera_update_interval_));
    }
    capture.release();
  }
}

bool Tello::getState()
{
  static bool first_time = true;
  if (first_time) {
    std::string msgs = stateRecv_->receiving(MSG_WAITFORONE);
    msgs = stateRecv_->receiving(MSG_WAITFORONE);
    first_time = false;
    return false;
  }
  std::string msgs = stateRecv_->receiving(MSG_WAITFORONE);
  if (msgs.empty()) {
    return false;
  }
  return parseState(msgs);
}

bool Tello::parseState(const std::string & data)
{
  std::vector<std::string> values, values_;
  values = split(data, ';');

  // if (values.size() < state.size()) {
  //   std::cout << "Error: Adding data to the 'state' attribute" << std::endl;
  //   return false;
  // }
  // if (values.size() > state.size()) {
  //   // get the last state.size() values
  //   values_ = std::vector<std::string>(values.end() - state.size(), values.end());
  //   for (auto& value : values_) {
  //     std::cout << value << std::endl;
  //   }
  // }
  // TODO(pariaspe): check if this is the best way to do this
  // int i = 0;
  try {
    for (auto & value : values) {
      if (value.size()) {
        values_ = split(value, ':');
        state_[values_[0]] = stod(values_[1]);
      }
    }
  } catch (const std::exception & e) {
    std::cout << "Error: Parsing state" << std::endl;
    return false;
  }
  return true;
}

void Tello::update()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  orientation_.x = state_["roll"] * M_PI / 180.0;
  orientation_.y = state_["pitch"] * M_PI / 180.0;
  orientation_.z = state_["yaw"] * M_PI / 180.0;

  velocity_.x = state_["vgx"] / 100.0;
  velocity_.y = state_["vgy"] / 100.0;
  velocity_.z = state_["vgz"] / 100.0;

  timeOF = state_["tof"];
  height_ = state_["h"] / 100.0;
  battery_ = static_cast<int>(state_["bat"]);
  timeMotor = state_["time"];
  barometer_ = state_["baro"] / 100.0;

  acceleration_.x = state_["agx"] * 9.81 / 1000.0;
  acceleration_.y = state_["agy"] * 9.81 / 1000.0;
  acceleration_.z = state_["agz"] * 9.81 / 1000.0;

  imu_[0] = orientation_;
  imu_[1] = velocity_;
  imu_[2] = acceleration_;
}

// Forward or backward move.
bool Tello::x_motion(double x)
{
  bool response = true;
  x *= 100.0;
  std::string msg;
  if (x > 0) {
    msg = "forward " + std::to_string(abs(x));
    response = sendCommand(msg);
  } else if (x < 0) {
    msg = "back " + std::to_string(abs(x));
    response = sendCommand(msg);
  }
  return response;
}

// right or left move.
bool Tello::y_motion(double y)
{
  bool response = true;
  y *= 100.0;
  std::string msg;
  if (y > 0) {
    msg = "right " + std::to_string(abs(y));
    response = sendCommand(msg);
  } else if (y < 0) {
    msg = "left " + std::to_string(abs(y));
    response = sendCommand(msg);
  }
  return response;
}

// up or left down.
bool Tello::z_motion(double z)
{
  bool response = true;
  std::string msg;
  z *= 100.0;
  if (z > 0) {
    msg = "up " + std::to_string(static_cast<int>(abs(z)));
    response = sendCommand(msg);
  } else if (z < 0) {
    msg = "down " + std::to_string(static_cast<int>(abs(z)));
    response = sendCommand(msg);
  }
  return response;
}

// clockwise or counterclockwise
// TODO(pariaspe): everything should be in rad units
bool Tello::yaw_twist(double yaw)
{
  bool response = true;
  std::string msg;
  if (yaw > 0) {
    msg = "cw " + std::to_string(abs(yaw));
    response = sendCommand(msg);
  } else if (yaw < 0) {
    msg = "ccw " + std::to_string(abs(yaw));
    response = sendCommand(msg);
  }
  return response;
}

// speed motion
bool Tello::speedMotion(double x, double y, double z, double yaw)
{
  bool response;
  std::string msg;

  /* x *= 100.0;           // cm/s
  y *= 100.0;           // cm/s
  z *= 100.0;           // cm/s
  yaw *= 180.0 / M_PI;  // degrees/s */

  // IN TELLO rc
  // x: left-right, y: forward-backward, z: up-down, yaw: clockwise-counterclockwise
  //
  x = std::clamp(x * 100.0, -100.0, 100.0);             // cm/s
  y = std::clamp(y * -100.0, -100.0, 100.0);            // cm/s (right is positive)
  z = std::clamp(z * 100.0, -100.0, 100.0);             // cm/s
  yaw = std::clamp(yaw * 180.0 / M_PI, -100.0, 100.0);  // degrees/s
                                                        //
  // std::cout << "x: " << x << " y: " << y << " z: " << z << " yaw: " << yaw << std::endl;

  msg = "rc " + std::to_string(static_cast<int>(y)) + " " + std::to_string(static_cast<int>(x)) +
    " " + std::to_string(static_cast<int>(z)) + " " + std::to_string(static_cast<int>(yaw));
  response = sendCommand(msg, false);
  return response;
}
