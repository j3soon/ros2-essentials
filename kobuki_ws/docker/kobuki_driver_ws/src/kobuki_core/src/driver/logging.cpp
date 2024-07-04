/**
 * @file src/driver/logging.cpp
 *
 * @brief Simple logging implementation.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "../../include/kobuki_core/logging.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki {

void logDebug(const std::string& message) {
  std::cout << ecl::green << "[DEBUG] " << message << ecl::reset << std::endl;
}

void logInfo(const std::string& message) {
  std::cout << "[INFO] " << message << std::endl;
}

void logWarning(const std::string& message) {
  std::cout << ecl::yellow << "[WARNING] " << message << ecl::reset << std::endl;
}

void logError(const std::string& message) {
  std::cout << ecl::red << "[ERROR] " << message << ecl::reset << std::endl;
}

} // namespace kobuki
