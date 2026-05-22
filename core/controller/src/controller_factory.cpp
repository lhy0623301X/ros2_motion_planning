/**
 * @file controller_factory.cpp
 * @brief Factory for internal controller algorithms.
 */
#include "controller_factory.h"

#include <algorithm>
#include <cctype>

#include "dwa_controller/dwa_controller.h"
#include "lqr_controller/lqr_controller.h"
#include "pid_controller/pid_controller.h"
#include "rpp_controller/rpp_controller.h"
#include "teb_controller/teb_controller.h"

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
  if (normalized_name == "rpp" || normalized_name == "rppcontroller") {
    return std::make_shared<RPPController>();
  }
  if (normalized_name == "dwa" || normalized_name == "dwacontroller") {
    return std::make_shared<DWAController>();
  }
  if (normalized_name == "teb" || normalized_name == "tebcontroller") {
    return std::make_shared<TEBController>();
  }
  return nullptr;
}

}  // namespace rmp::controller
