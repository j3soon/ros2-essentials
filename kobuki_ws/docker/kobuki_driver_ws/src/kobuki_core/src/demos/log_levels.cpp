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
#include <ecl/time.hpp>
#include <ecl/command_line.hpp>

#include <kobuki_core/kobuki.hpp>

/*****************************************************************************
 ** Main
 *****************************************************************************/

int main(int argc, char **argv)
{
  ecl::CmdLine cmd_line("log_levels", ' ', "0.1");
  ecl::ValueArg<std::string> device_port(
      "p", "port",
      "Path to device file of serial port to open",
      false,
      "/dev/kobuki",
      "string"
  );
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);

  std::cout << ecl::bold << "\nLog Levels Demo\n" << ecl::reset << std::endl;

  kobuki::Parameters parameters;
  parameters.device_port = device_port.getValue();
  parameters.log_level = kobuki::LogLevel::DEBUG;

  kobuki::Kobuki kobuki;
  try {
    kobuki.init(parameters);
  } catch (ecl::StandardException &e) {
    std::cout << e.what();
  }

  ecl::Sleep()(5);
  return 0;
}
