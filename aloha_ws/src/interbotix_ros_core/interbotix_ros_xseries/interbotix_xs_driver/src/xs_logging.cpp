// Copyright 2022 Trossen Robotics
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
//    * Neither the name of the the copyright holder nor the names of its
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

#include "interbotix_xs_driver/xs_logging.hpp"

#include <stdio.h>
#include <stdlib.h>

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>


namespace interbotix_xs
{

namespace logging
{

#define OFF "\x1B[0m"
#define END OFF "\n"
#define GRN "\x1B[32m"
#define YLW "\x1B[33m"
#define RED "\x1B[31m"

void log(logging::Level level, const char * fmt, ...)
{
  if (level < _level) {
    return;
  }

  std::string msg = "";
  switch (level) {
    case Level::DEBUG:
      msg += GRN;
      msg += "[DEBUG] ";
      break;
    case Level::INFO:
      msg += OFF;
      msg += "[INFO] ";
      break;
    case Level::WARN:
      msg += YLW;
      msg += "[WARN] ";
      break;
    case Level::ERROR:
      msg += RED;
      msg += "[ERROR] ";
      break;
    case Level::FATAL:
      msg += RED;
      msg += "[FATAL] ";
      break;
    default:
      break;
  }

  auto duration = std::chrono::system_clock::now().time_since_epoch();
  double seconds = std::chrono::duration<double>(duration).count();

  msg = msg +
    "[" +
    std::to_string(seconds) +
    "] ";

  va_list args;
  va_start(args, fmt);
  size_t size = 1024;
  std::vector<char> dynamicbuf(size);
  char * buf = &dynamicbuf[0];

  va_list argsTmp;

  while (1) {
    va_copy(argsTmp, args);
    int needed = vsnprintf(buf, size, fmt, argsTmp);
    va_end(argsTmp);
    if (needed < static_cast<int>(size) - 1 && needed >= 0) {
      msg.append(buf, static_cast<size_t>(needed));
      break;
    }
    size = needed >= 0 ? needed + 2 : size * 2;
    dynamicbuf.resize(size);
    buf = &dynamicbuf[0];
  }
  va_end(args);
  msg.append(END);
  std::cerr << msg.c_str();
}

void set_level(Level level)
{
  _level = level;
  XSLOG_DEBUG("Set logging level to %d.", level);
}

void set_level(std::string level)
{
  if (level == "DEBUG") {
    set_level(Level::DEBUG);
  } else if (level == "INFO") {
    set_level(Level::INFO);
  } else if (level == "WARN") {
    set_level(Level::WARN);
  } else if (level == "ERROR") {
    set_level(Level::ERROR);
  } else if (level == "FATAL") {
    set_level(Level::FATAL);
  } else {
    XSLOG_ERROR("Tried to set an invalid logging level.");
  }
}

Level get_level()
{
  return _level;
}

}  // namespace logging


}  // namespace interbotix_xs
