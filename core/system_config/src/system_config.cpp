/**
 * *********************************************************
 *
 * @file: system_config.cpp
 * @brief: Contains the system configure module
 * @author: Yang Haodong
 * @date: 2025-12-5
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "common/util/log.h"
#include "system_config/system_config.h"

namespace rmp::system_config {
/**
 * @brief Load and parse the system configuration from the file.
 */
void SystemConfig::initialize() {
  std::string pkg_share = ament_index_cpp::get_package_share_directory("system_config");
  std::string config_path = pkg_share + "/system_config/system_config.pb.txt";
  if (readTextProtoFile(config_path, &config_)) {
    is_initialized_ = true;
  } else {
    R_WARN << "Read System configure file " << config_path << " failed.";
  }
}

/**
 * @brief Return the loaded system configuration, initializing on first access.
 * @return Reference to the in-memory SystemConfig protobuf.
 */
const pb::SystemConfig& SystemConfig::configure() {
  if (!is_initialized_) {
    initialize();
  }

  return config_;
}

}  // namespace rmp