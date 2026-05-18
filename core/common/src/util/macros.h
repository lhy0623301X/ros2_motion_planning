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
#include <memory>
#include <mutex>

// Macro to check if pointer is not null (replaces CHECK_NOTNULL)
// Only define if not already defined by glog
#ifndef CHECK_NOTNULL
#define CHECK_NOTNULL(ptr) \
  ((ptr) == nullptr ? (assert(false), (ptr)) : (ptr))
#endif

// NOTE:
// Do NOT define CHECK/CHECK_* macros here. In this ROS2 port we provide
// stream-friendly CHECK macros in `common/util/log.h` (so `CHECK(x) << msg`
// works). Defining them here as plain `assert()` would break those usages.

// Macro to return early if pointer is null (replaces RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)          \
  do {                                \
    if ((ptr) == nullptr) {          \
      return;                          \
    }                                 \
  } while (0)

#define RETURN_VAL_IF_NULL(ptr, val)  \
  do {                                \
    if ((ptr) == nullptr) {          \
      return (val);                  \
    }                                 \
  } while (0)

#define RETURN_IF(condition)          \
  do {                                \
    if (condition) {                  \
      return;                         \
    }                                 \
  } while (0)

#define RETURN_VAL_IF(condition, val) \
  do {                                \
    if (condition) {                  \
      return (val);                   \
    }                                 \
  } while (0)

// Multi-thread lock macro (replaces UNIQUE_LOCK_MULTITHREAD from Apollo)
// In ROS2, we always use multi-threading, so this just creates a lock
#define UNIQUE_LOCK_MULTITHREAD(mutex_type) \
  std::unique_lock<std::mutex> lock(mutex_type)

#define UNUSED(param) (void)(param)

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

// Singleton pattern macro
#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname *Instance(bool create_if_needed = true) {              \
    static classname *instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
  static void CleanUp() {                                                 \
    auto instance = Instance(false);                                      \
    if (instance != nullptr) {                                            \
      delete instance;                                                     \
      instance = nullptr;                                                  \
    }                                                                     \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  DISALLOW_COPY_AND_ASSIGN(classname);

