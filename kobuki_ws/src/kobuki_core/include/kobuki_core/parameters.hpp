/**
 * @file include/kobuki_core/parameters.hpp
 *
 * @brief Parameter configuration for the kobuki.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_CORE_PARAMETERS_HPP_
#define KOBUKI_CORE_PARAMETERS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>

#include "logging.hpp"
#include "modules/battery.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief Parameter list and validator for the kobuki.
 */
class Parameters
{
public:
  Parameters() :
    device_port("/dev/kobuki"),
    sigslots_namespace("/kobuki"),
    simulation(false),
    enable_acceleration_limiter(true),
    battery_capacity(Battery::capacity),
    battery_low(Battery::low),
    battery_dangerous(Battery::dangerous),
    linear_acceleration_limit(0.3),
    linear_deceleration_limit(-0.3*1.2),
    angular_acceleration_limit(3.5),
    angular_deceleration_limit(-3.5*1.2),
    log_level(LogLevel::WARNING)
  {
  } /**< @brief Default constructor. **/

  std::string device_port;         /**< @brief The serial device port name [/dev/kobuki] **/
  std::string sigslots_namespace;  /**< @brief The first part of a sigslot connection namespace ["/kobuki"] **/
  bool simulation;                 /**< @brief Whether to put the motors in loopback mode or not [false] **/
  bool enable_acceleration_limiter;/**< @brief Enable or disable the acceleration limiter [true] **/
  double battery_capacity;         /**< @brief Capacity voltage of the battery [16.5V] **/ /* defaults defined in battery.cpp */
  double battery_low;              /**< @brief Threshold for battery level warnings [14.0V] **/  /* defaults defined in battery.cpp */
  double battery_dangerous;        /**< @brief Threshold for battery level in danger of depletion [13.2V] **/  /* defaults defined in battery.cpp */

  double linear_acceleration_limit;
  double linear_deceleration_limit;
  double angular_acceleration_limit;
  double angular_deceleration_limit;

  LogLevel log_level;              /**< @brief Print to stdout messages for this logging level and above. **/

  /**
   * @brief A validator to ensure the user has supplied correct/sensible parameter values.
   *
   * This validates the current parameters and if invalid, puts an error string in error_msg.
   *
   * @return bool : true if valid, false otherwise.
   */
  bool validate()
  {
    // not doing anything right now -  delete it, if we can find a use case ...
    return true;
  }

  std::string error_msg; /**< @brief Provides error messages when parameter validation fails (internal purposes only) **/
};

} // namespace kobuki

#endif /* KOBUKI_CORE_PARAMETERS_HPP_ */
