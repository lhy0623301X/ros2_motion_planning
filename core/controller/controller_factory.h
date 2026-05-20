/**
 * @file controller_factory.h
 * @brief Factory for internal controller algorithms.
 */
#ifndef RMP_CONTROLLER_CONTROLLER_FACTORY_H_
#define RMP_CONTROLLER_CONTROLLER_FACTORY_H_

#include <memory>
#include <string>

#include "controller_algorithm.h"

namespace rmp::controller {

class ControllerFactory
{
public:
  static std::shared_ptr<ControllerAlgorithm> create(const std::string & controller_name);
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_CONTROLLER_FACTORY_H_
