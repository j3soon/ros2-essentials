/**
 * @file src/demos/logging.cpp
 *
 * @brief Example program for custom logging with kobuki.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream>
#include <string>

#include <ecl/console.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/time.hpp>
#include <ecl/command_line.hpp>

#include <kobuki_core/kobuki.hpp>

/*****************************************************************************
 ** Classes
 *****************************************************************************/

class KobukiManager
{
public:
  KobukiManager(const std::string &device) :
    slot_debug(&KobukiManager::logCustomDebug, *this),
    slot_info(&KobukiManager::logCustomInfo, *this),
    slot_warning(&KobukiManager::logCustomWarning, *this),
    slot_error(&KobukiManager::logCustomError, *this)
  {
    kobuki::Parameters parameters;

    parameters.device_port = device;
    // Disable the default loggers
    parameters.log_level = kobuki::LogLevel::NONE;

    // Wire them up ourselves
    slot_debug.connect(parameters.sigslots_namespace + "/debug");
    slot_info.connect(parameters.sigslots_namespace + "/info");
    slot_warning.connect(parameters.sigslots_namespace + "/warning");
    slot_error.connect(parameters.sigslots_namespace + "/error");

    try {
      kobuki.init(parameters);
    } catch (ecl::StandardException &e) {
      std::cout << e.what();
    }
  }

  void logCustomDebug(const std::string &message)
  {
    std::cout << ecl::green << "[DEBUG_WITH_COLANDERS] " << message << ecl::reset << std::endl;
  }

  void logCustomInfo(const std::string &message)
  {
    std::cout << "[INFO_WITH_COLANDERS] " << message << ecl::reset << std::endl;
  }

  void logCustomWarning(const std::string &message)
  {
    std::cout << ecl::yellow << "[WARNING_WITH_COLANDERS] " << message << ecl::reset << std::endl;
  }

  void logCustomError(const std::string &message) {
    std::cout << ecl::red << "[ERROR_WITH_COLANDERS] " << message << ecl::reset << std::endl;
  }

private:
  kobuki::Kobuki kobuki;
  ecl::Slot<const std::string &> slot_debug, slot_info, slot_warning, slot_error;
};

/*****************************************************************************
 ** Main
 *****************************************************************************/

int main(int argc, char **argv)
{
  ecl::CmdLine cmd_line("logging", ' ', "0.3");
  ecl::ValueArg<std::string> device_port(
      "p", "port",
      "Path to device file of serial port to open",
      false,
      "/dev/kobuki",
      "string"
  );
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);

  std::cout << ecl::bold << "\nLogging Demo\n" << ecl::reset << std::endl;

  KobukiManager kobuki_manager(device_port.getValue());
  ecl::Sleep()(5);
  return 0;
}
