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
 * @file socket_udp.cpp
 *
 * Implements socket communication.
 *
 * @authors Daniel Fernández Sánchez
 */

#include "socket_udp.hpp"

// Custom exception class for socket errors
class SocketException : public std::runtime_error {
public:
    SocketException(const std::string& message) : std::runtime_error(message) {}
};

SocketUdp::SocketUdp(const std::string & host, int port, int local_port, uint bufferSize)
{
  (void)bufferSize;
  host_ = host;
  port_ = port;
  std::cout << "Creating socket ... " << host_ << ":" << port_;
  if(local_port != 0) {
    std::cout << " <- '':" << local_port << std::endl;
  } else {
    std::cout << std::endl;
  }

  /* socket: create the socket */
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    std::cout << "Error opening socket..." << std::endl;
  }

  /* socket: set reusable */
  const int enable = 1;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
    std::cout << "setsockopt(SO_REUSEADDR) failed" << std::endl;
  }

  /* build the server's Internet address */
  serv_addr_.sin_port = htons(port_);
  serv_addr_.sin_addr.s_addr = inet_addr(host_.c_str());
  serv_addr_.sin_family = AF_INET;

  if(local_port != 0) {
    // Set up the local address and port
    local_addr_.sin_family = AF_INET;
    local_addr_.sin_addr.s_addr = INADDR_ANY;  // Bind to any local address
    local_addr_.sin_port = htons(local_port);       // Local port to send from

    // Bind the socket to the local address and port
    if (bind(socket_fd_, (const struct sockaddr *)&local_addr_, sizeof(local_addr_)) < 0) {
        close(socket_fd_);
        throw SocketException("Bind failed: " + std::string(strerror(errno)));
    }
  }

  /* build the sending destination addres */
  if (!setDestAddr()) {
    std::cout << "Unable to setDestAddr" << std::endl;
  }
}

SocketUdp::~SocketUdp()
{
  std::cout << "closing socket ..." << std::endl;
  close(socket_fd_);
}

bool SocketUdp::setDestAddr()
{
  addrinfo * addrInfo{nullptr};
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  std::string port_str = std::to_string(port_);
  int ret = getaddrinfo(host_.c_str(), port_str.c_str(), &hints, &addrInfo);
  if (ret != 0) {
    std::cout << "Error: setting dest_addr sockaddr_storage" << std::endl;
    return false;
  }
  memcpy(&dest_addr_, addrInfo->ai_addr, addrInfo->ai_addrlen);
  freeaddrinfo(addrInfo);
  return true;
}

bool SocketUdp::bindServer()
{
  std::cout << "Server binding to " << host_ << ":" << port_ << std::endl;
  int ret = bind(socket_fd_, reinterpret_cast<sockaddr *>(&serv_addr_), sizeof(serv_addr_));
  if (ret < 0) {
    std::cout << "Unable to bind." << std::endl;
    return false;
  }
  return true;
}

bool SocketUdp::sending(std::string message)
{
  const std::vector<unsigned char> msgs{std::cbegin(message), std::cend(message)};
  const socklen_t dest_addr_len{sizeof(dest_addr_)};

  int n = sendto(
    socket_fd_, msgs.data(), msgs.size(), 0, reinterpret_cast<sockaddr *>(&dest_addr_),
    dest_addr_len);
  if (n < 0) {
    std::cout << "sending: It has been impossible to send the message " << message << std::endl;
    return false;
  }
  return true;
}

std::string SocketUdp::receiving(const int flags)
{
  std::string msg;
  socklen_t serv_addr_len{sizeof(dest_addr_)};
  int n = recvfrom(
    socket_fd_, buffer_.data(), buffer_.size(), flags,
    reinterpret_cast<sockaddr *>(&dest_addr_), &serv_addr_len);

  if (n < 1) {
    return "";
  }

  msg.append(buffer_.cbegin(), buffer_.cbegin() + n);
  msg = msg.erase(msg.find_last_not_of(" \n\r\t") + 1);
  return msg;
}
