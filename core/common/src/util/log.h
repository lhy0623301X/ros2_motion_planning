/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <cassert>
#include <atomic>
#include <iostream>
#include <sstream>
#include <cstring>  // 添加这个头文件用于 strrchr

#include <rclcpp/logging.hpp>

#include "common/util/macros.h"

// Helper class to support stream output for ROS2 logging (like glog)
namespace apollo {
namespace common {
namespace util {

inline std::atomic_bool & LogEnabledFlag()
{
  static std::atomic_bool enabled{false};
  return enabled;
}

inline void SetLogEnabled(bool enabled)
{
  LogEnabledFlag().store(enabled, std::memory_order_relaxed);
}

inline bool IsLogEnabled()
{
  return LogEnabledFlag().load(std::memory_order_relaxed);
}

class LogStream {
 public:
  enum Level {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
  };
  
  // 修改构造函数，添加文件名和行号参数
  explicit LogStream(Level level)
      : level_(level),
        logger_(rclcpp::get_logger("hric_local_routing")) {}

  template <typename T>
  LogStream& operator<<(const T& value) {
    message_ << value;
    return *this;
  }
  
  // Specialization for std::pair to support pair output
  template <typename T1, typename T2>
  LogStream& operator<<(const std::pair<T1, T2>& value) {
    message_ << "(" << value.first << ", " << value.second << ")";
    return *this;
  }

  // Handle std::endl specifically
  LogStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
    if (manip == static_cast<std::ostream& (*)(std::ostream&)>(std::endl)) {
      message_ << '\n';
    } else {
      message_ << manip;
    }
    return *this;
  }

  ~LogStream() {
    
    // RCLCPP_*_STREAM expects an expression that can be << into stringstream
    // We need to use the stringstream's str() result directly
    switch (level_) {
      case DEBUG:
        RCLCPP_DEBUG_STREAM(logger_, message_.str());
        break;
      case INFO:
        RCLCPP_INFO_STREAM(logger_, message_.str());
        break;
      case WARN:
        RCLCPP_WARN_STREAM(logger_, message_.str());
        break;
      case ERROR:
        RCLCPP_ERROR_STREAM(logger_, message_.str());
        break;
      case FATAL:
        RCLCPP_FATAL_STREAM(logger_, message_.str());
        break;
    }
  }

 private:
  Level level_;
  rclcpp::Logger logger_;
  std::ostringstream message_;
};

}  // namespace util
}  // namespace common
}  // namespace apollo

// Replace Apollo cyber log macros with ROS2 logging
// Use helper class to support << operator like glog
#define AINFO \
  if (!::apollo::common::util::IsLogEnabled()) ; else \
    ::apollo::common::util::LogStream(::apollo::common::util::LogStream::INFO)
#define ADEBUG \
  if (!::apollo::common::util::IsLogEnabled()) ; else \
    ::apollo::common::util::LogStream(::apollo::common::util::LogStream::DEBUG)
#define AWARN \
  if (!::apollo::common::util::IsLogEnabled()) ; else \
    ::apollo::common::util::LogStream(::apollo::common::util::LogStream::WARN)
#define AERROR \
  if (!::apollo::common::util::IsLogEnabled()) ; else \
    ::apollo::common::util::LogStream(::apollo::common::util::LogStream::ERROR)
#define AFATAL \
  if (!::apollo::common::util::IsLogEnabled()) ; else \
    ::apollo::common::util::LogStream(::apollo::common::util::LogStream::FATAL)

// Replace ACHECK with a macro that supports stream output (like glog CHECK)
// Usage: ACHECK(condition) << "error message";
namespace apollo {
namespace common {
namespace util {

// Forward declaration
class CheckOpVoidify;

// Helper classes for ACHECK stream output
class CheckOpStream {
  friend class CheckOpVoidify;
 public:
  CheckOpStream(rclcpp::Logger logger, const char* file, int line,
                const char* cond)
      : logger_(logger), file_(file), line_(line), cond_(cond) {}

  template <typename T>
  CheckOpStream& operator<<(const T& value) {
    message_ << value;
    return *this;
  }
  
  // Handle std::endl specifically
  CheckOpStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
    if (manip == static_cast<std::ostream& (*)(std::ostream&)>(std::endl)) {
      message_ << '\n';
    } else {
      message_ << manip;
    }
    return *this;
  }

  ~CheckOpStream() {
    std::ostringstream full_msg;
    full_msg << "[" << file_ << ":" << line_ << "] Assertion failed: "
             << cond_ << " - " << message_.str();
    RCLCPP_FATAL_STREAM(logger_, full_msg.str());
    assert(false);
  }

 private:
  rclcpp::Logger logger_;
  const char* file_;
  int line_;
  const char* cond_;
  std::ostringstream message_;
};

class CheckOpVoidify {
 public:
  // Return reference to allow chaining with << operator
  // Store temporary in static thread_local to extend lifetime
  CheckOpStream& operator&(CheckOpStream&& stream) { 
    thread_local static CheckOpStream* stored_stream = nullptr;
    // Delete old stream if exists
    if (stored_stream) {
      delete stored_stream;
    }
    // Create new stream with moved parameters
    stored_stream = new CheckOpStream(
        stream.logger_, stream.file_, stream.line_, stream.cond_);
    // Copy message content
    stored_stream->message_ << stream.message_.str();
    return *stored_stream;
  }
  // Const version for compatibility
  void operator&(const CheckOpStream&) {}
};

// Helper class to enable stream output for CHECK macros
// This class stores a CheckOpStream and forwards << operations to it
class CheckOpHelper {
 public:
  CheckOpHelper(rclcpp::Logger logger, const char* file, int line, const char* cond)
      : stream_(logger, file, line, cond) {}
  
  template<typename T>
  CheckOpHelper& operator<<(const T& value) {
    stream_ << value;
    return *this;
  }
  
  // Handle std::endl specifically
  CheckOpHelper& operator<<(std::ostream& (*manip)(std::ostream&)) {
    stream_ << manip;
    return *this;
  }
  
  // No conversion operator needed - we'll use a different macro design
  
  ~CheckOpHelper() {
    // Destructor will trigger CheckOpStream's destructor which logs the error
  }
  
 private:
  CheckOpStream stream_;
};

}  // namespace util
}  // namespace common
}  // namespace apollo

#define ACHECK(cond) \
  if (!(cond)) ::apollo::common::util::CheckOpHelper( \
                   rclcpp::get_logger("hric_local_routing"), __FILE__, __LINE__, #cond)

// AERROR_IF macro: conditionally log error
#define AERROR_IF(cond) \
  if (cond) AERROR

// CHECK_NOTNULL macro (only define if not already defined in macros.h)
#ifndef CHECK_NOTNULL
#define CHECK_NOTNULL(val) ACHECK((val) != nullptr)
#endif

// Additional CHECK macros for compatibility (only define if not already defined in macros.h)
// These macros support stream output like glog
#ifndef CHECK
#define CHECK(cond) \
  if (!(cond)) ::apollo::common::util::CheckOpHelper( \
                   rclcpp::get_logger("hric_local_routing"), __FILE__, __LINE__, #cond)
#endif
#ifndef CHECK_EQ
#define CHECK_EQ(a, b) \
  if (!((a) == (b))) ::apollo::common::util::CheckOpHelper( \
                   rclcpp::get_logger("hric_local_routing"), __FILE__, __LINE__, #a " == " #b)
#endif
#ifndef CHECK_NE
#define CHECK_NE(a, b) \
  if (!((a) != (b))) ::apollo::common::util::CheckOpHelper( \
                   rclcpp::get_logger("hric_local_routing"), __FILE__, __LINE__, #a " != " #b)
#endif
#ifndef CHECK_GE
#define CHECK_GE(a, b) \
  if (!((a) >= (b))) ::apollo::common::util::CheckOpHelper( \
                   rclcpp::get_logger("hric_local_routing"), __FILE__, __LINE__, #a " >= " #b)
#endif
#ifndef CHECK_LE
#define CHECK_LE(a, b) \
  if (!((a) <= (b))) ::apollo::common::util::CheckOpHelper( \
                   rclcpp::get_logger("hric_local_routing"), __FILE__, __LINE__, #a " <= " #b)
#endif
#ifndef CHECK_GT
#define CHECK_GT(a, b) \
  if (!((a) > (b))) ::apollo::common::util::CheckOpHelper( \
                   rclcpp::get_logger("hric_local_routing"), __FILE__, __LINE__, #a " > " #b)
#endif
#ifndef CHECK_LT
#define CHECK_LT(a, b) \
  if (!((a) < (b))) ::apollo::common::util::CheckOpHelper( \
                   rclcpp::get_logger("hric_local_routing"), __FILE__, __LINE__, #a " < " #b)
#endif
// DCHECK macros (debug-only checks, same as CHECK in release builds)
#ifndef DCHECK
#define DCHECK(cond) ACHECK(cond)
#endif
#ifndef DCHECK_EQ
#define DCHECK_EQ(a, b) ACHECK((a) == (b))
#endif
#ifndef DCHECK_NE
#define DCHECK_NE(a, b) ACHECK((a) != (b))
#endif
#ifndef DCHECK_GE
#define DCHECK_GE(a, b) ACHECK((a) >= (b))
#endif
#ifndef DCHECK_LE
#define DCHECK_LE(a, b) ACHECK((a) <= (b))
#endif
#ifndef DCHECK_GT
#define DCHECK_GT(a, b) ACHECK((a) > (b))
#endif
#ifndef DCHECK_LT
#define DCHECK_LT(a, b) ACHECK((a) < (b))
#endif
