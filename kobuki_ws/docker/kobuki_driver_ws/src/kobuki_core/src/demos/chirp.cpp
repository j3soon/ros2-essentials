/**
 * @file src/demos/chirp.cpp
 *
 * @brief Example program for kobuki initialisation.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream>
#include <string>

#include <ecl/time.hpp>
#include <ecl/command_line.hpp>

#include <kobuki_core/kobuki.hpp>

/*****************************************************************************
 ** Classes
 *****************************************************************************/

class KobukiManager
{
public:
  KobukiManager(const std::string &device)
  {
    kobuki::Parameters parameters;
    // Specify the device port, default: /dev/kobuki
    parameters.device_port = device;

    // Other parameters are typically happy enough as defaults, some examples follow
    //
    // namespaces all sigslot connection names, default: /kobuki
    parameters.sigslots_namespace = "/kobuki";
    // Most use cases will bring their own smoothing algorithms, but if
    // you wish to utilise kobuki's minimal acceleration limiter, set to true
    parameters.enable_acceleration_limiter = false;
    // Adjust battery thresholds if your levels are significantly varying from factory settings.
    // This will affect led status as well as triggering driver signals
    parameters.battery_capacity = 16.5;
    parameters.battery_low = 14.0;
    parameters.battery_dangerous = 13.2;

    // Initialise - exceptions are thrown if parameter validation or initialisation fails.
    try {
      kobuki.init(parameters);
    } catch (ecl::StandardException &e) {
      std::cout << e.what();
    }
  }
private:
  kobuki::Kobuki kobuki;
};

/*****************************************************************************
 ** Main
 *****************************************************************************/

int main(int argc, char **argv)
{
  ecl::CmdLine cmd_line("chirp", ' ', "0.2");
  ecl::ValueArg<std::string> device_port(
      "p", "port",
      "Path to device file of serial port to open",
      false,
      "/dev/kobuki",
      "string"
  );
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);

  KobukiManager kobuki_manager(device_port.getValue());
  ecl::Sleep()(5);
  return 0;
}
