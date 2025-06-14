/**
 * @file src/demos/stream.cpp
 *
 * @brief Example program for streaming kobuki's sensors.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream>
#include <string>

#include <ecl/command_line.hpp>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>

#include <kobuki_core/kobuki.hpp>

/*****************************************************************************
 ** Classes
 *****************************************************************************/

class KobukiManager
{
public:
  KobukiManager(const std::string &device) :
      slot_stream_data(&KobukiManager::processStreamData, *this)
  {
    kobuki::Parameters parameters;
    parameters.device_port = device;

    try {
      kobuki.init(parameters);
    } catch (ecl::StandardException &e) {
      std::cout << e.what();
    }
    slot_stream_data.connect("/kobuki/stream_data");
  }

  /*
   * Nothing to do in the main thread, just put it to sleep
   */
  void spin()
  {
    ecl::Sleep sleep(1);
    while (true) {
      sleep();
    }
  }

  /*
   * Called whenever the kobuki receives a data packet.
   * Up to you from here to process it.
   */
  void processStreamData()
  {
    kobuki::CoreSensors::Data data = kobuki.getCoreSensorData();
    std::cout << "[" << ecl::TimeStamp() << "] Encoders [" << data.left_encoder << "," << data.right_encoder << "]" << std::endl;
  }

private:
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
};

/*****************************************************************************
 ** Main
 *****************************************************************************/

int main(int argc, char **argv)
{
  ecl::CmdLine cmd_line("buttons", ' ', "0.1");
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
  kobuki_manager.spin();
  return 0;
}
