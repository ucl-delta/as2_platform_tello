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
 * @file socket_udp.hpp
 *
 * Implements socket communication.
 *
 * @authors Daniel Fernández Sánchez
 */

#ifndef SOCKET_UDP_HPP_
#define SOCKET_UDP_HPP_

#include <arpa/inet.h>   // For inet_addr()
#include <netdb.h>       // For gethostbyname()
#include <netinet/in.h>  // For sockaddr_in
#include <string.h>
#include <sys/socket.h>  // For socket(), connect(), send(), and recv()
#include <sys/types.h>   // For data types
#include <unistd.h>      // For close()

#include <stdio.h>

#include <cstring>  // memcpy
#include <sstream>

#include <array>
#include <iostream>
#include <string>
#include <vector>

class SocketUdp
{
private:
  int socket_fd_;
  std::string host_;
  int port_;

  sockaddr_in serv_addr_;
  sockaddr_in local_addr_;
  std::array<unsigned char, 1024> buffer_;
  sockaddr_storage dest_addr_;

private:
  bool setDestAddr();

public:
  explicit SocketUdp(const std::string & host = "0.0.0.0", int port = 0, int local_port = 0, uint bufferSize = 1024);
  ~SocketUdp();  // closing socket

  bool bindServer();
  inline int getSocketfd() const {return socket_fd_;}
  inline const char * getIP() const {return host_.c_str();}
  inline int getPort() const {return port_;}

  inline void setSocketfd(int socket_fd) {this->socket_fd_ = socket_fd;}
  inline void setIP(const char * host) {this->host_ = host;}
  inline void setPort(int port) {this->port_ = port;}

  bool sending(std::string message);
  std::string receiving(const int flags = MSG_DONTWAIT);
};

#endif  // SOCKET_UDP_HPP_
