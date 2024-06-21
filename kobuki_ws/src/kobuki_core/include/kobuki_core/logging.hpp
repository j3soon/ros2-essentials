/**
 * @file include/kobuki_core/logging.hpp
 *
 * @brief Log levels and simple logging to screen.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_LOGGING_HPP_
#define KOBUKI_LOGGING_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream>
#include <string>

#include <ecl/console.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki {

/*****************************************************************************
 ** Log Levels
 *****************************************************************************/
/**
 * @brief Internal logging levels.
 *
 * Kobuki will log to stdout the specified log level and higher. For example
 * if WARNING is specified, it will log both warning and error messages. To
 * disable logging, use NONE.
 *
 * To connect to your own logging infrastructure, use NONE and provide slots
 * (callbacks) to the kobuki debug, info, warning and error signals.
 */
enum LogLevel {
  DEBUG = 0,
  INFO = 1,
  WARNING = 2,
  ERROR = 3,
  NONE = 4
};

void logDebug(const std::string& message);
void logInfo(const std::string& message);
void logWarning(const std::string& message);
void logError(const std::string& message);

} // namespace kobuki

#endif /* KOBUKI_LOGGING_HPP_ */
