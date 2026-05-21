/**
 * @file controller_factory.cpp
 * @brief Factory for internal controller algorithms.
 */
#include "controller_factory.h"

#include <algorithm>
#include <cctype>

#include "lqr_controller/lqr_controller.h"
#include "pid_controller/pid_controller.h"

namespace rmp::controller {

namespace {

std::string normalizeName(std::string name)
{
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  name.erase(std::remove(name.begin(), name.end(), ' '), name.end());
  name.erase(std::remove(name.begin(), name.end(), '_'), name.end());
  name.erase(std::remove(name.begin(), name.end(), '-'), name.end());
  return name;
}

}  // namespace

std::shared_ptr<ControllerAlgorithm> ControllerFactory::create(
  const std::string & controller_name)
{
  const auto normalized_name = normalizeName(controller_name);
  if (normalized_name == "pid" || normalized_name == "pidcontroller") {
    return std::make_shared<PIDController>();
  }
  if (normalized_name == "lqr" || normalized_name == "lqrcontroller") {
    return std::make_shared<LQRController>();
  }
  return nullptr;
}

}  // namespace rmp::controller
