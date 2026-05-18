/**
 * *********************************************************
 *
 * @file: log.h
 * @brief: Contains logger
 * @author: Yang Haodong
 * @date: 2024-09-24
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <atomic>

#include <glog/logging.h>
#include <glog/raw_logging.h>

#ifndef RMP_COMMON_UTIL_LOG_H_
#define RMP_COMMON_UTIL_LOG_H_

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

}  // namespace util
}  // namespace common

#define ADEBUG \
  if (!::common::util::IsLogEnabled()) ; else VLOG(4) << "[DEBUG] "
#define AINFO \
  if (!::common::util::IsLogEnabled()) ; else LOG(INFO)
#define AWARN \
  if (!::common::util::IsLogEnabled()) ; else LOG(WARNING)
#define AERROR \
  if (!::common::util::IsLogEnabled()) ; else LOG(ERROR)
#define AFATAL \
  if (!::common::util::IsLogEnabled()) ; else LOG(FATAL)

#ifndef R_DEBUG
#define R_DEBUG ADEBUG
#endif
#ifndef R_INFO
#define R_INFO AINFO
#endif
#ifndef R_WARN
#define R_WARN AWARN
#endif
#ifndef R_ERROR
#define R_ERROR AERROR
#endif
#ifndef R_FATAL
#define R_FATAL AFATAL
#endif

// LOG_IF
#define AINFO_IF(cond) LOG_IF(INFO, cond)
#define AERROR_IF(cond) LOG_IF(ERROR, cond)
#define ACHECK(cond) CHECK(cond)

#ifndef R_INFO_IF
#define R_INFO_IF(cond) AINFO_IF(cond)
#endif
#ifndef R_ERROR_IF
#define R_ERROR_IF(cond) AERROR_IF(cond)
#endif
#ifndef R_CHECK
#define R_CHECK(cond) ACHECK(cond)
#endif

// LOG_EVERY_N
#define AINFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define AWARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define AERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#ifndef R_INFO_EVERY
#define R_INFO_EVERY(freq) AINFO_EVERY(freq)
#endif
#ifndef R_WARN_EVERY
#define R_WARN_EVERY(freq) AWARN_EVERY(freq)
#endif
#ifndef R_ERROR_EVERY
#define R_ERROR_EVERY(freq) AERROR_EVERY(freq)
#endif

#define RETURN_IF_NULL(ptr)                                                                                            \
  if (ptr == nullptr)                                                                                                  \
  {                                                                                                                    \
    AWARN << #ptr << " is nullptr.";                                                                                   \
    return;                                                                                                            \
  }

#define RETURN_VAL_IF_NULL(ptr, val)                                                                                   \
  if (ptr == nullptr)                                                                                                  \
  {                                                                                                                    \
    AWARN << #ptr << " is nullptr.";                                                                                   \
    return val;                                                                                                        \
  }

#define RETURN_IF(condition)                                                                                           \
  if (condition)                                                                                                       \
  {                                                                                                                    \
    AWARN << #condition << " is not met.";                                                                             \
    return;                                                                                                            \
  }

#define RETURN_VAL_IF(condition, val)                                                                                  \
  if (condition)                                                                                                       \
  {                                                                                                                    \
    AWARN << #condition << " is not met.";                                                                             \
    return val;                                                                                                        \
  }

namespace rmp::common::util
{
class LoggerInitializer
{
public:
  LoggerInitializer()
  {
    google::InitGoogleLogging("ros motion planning library logger");
    google::SetStderrLogging(google::INFO);
    FLAGS_colorlogtostderr = true;
  }

  ~LoggerInitializer()
  {
    google::ShutdownGoogleLogging();
  }
};
}  // namespace rmp
extern rmp::common::util::LoggerInitializer logger_initializer;

#endif  // APOLLO_COMMON_LOG_H_
