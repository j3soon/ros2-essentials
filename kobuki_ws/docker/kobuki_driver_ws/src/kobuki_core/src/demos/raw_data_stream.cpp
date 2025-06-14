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
    slot_raw_data_stream(&KobukiManager::logRawDataStream, *this)
  {
    kobuki::Parameters parameters;

    parameters.device_port = device;

    slot_raw_data_stream.connect(parameters.sigslots_namespace + "/raw_data_stream");

    try {
      kobuki.init(parameters);
    } catch (ecl::StandardException &e) {
      std::cout << e.what();
    }
  }

  void logRawDataStream(kobuki::PacketFinder::BufferType& buffer)
  {
    std::ostringstream ostream;
    ostream << ecl::cyan << "[" << ecl::TimeStamp() << "] " << ecl::yellow;
    ostream << std::setfill('0') << std::uppercase;
    for (unsigned int i = 0; i < buffer.size(); i++) {
      ostream << std::hex << std::setw(2) << static_cast<unsigned int>(buffer[i]) << " " << std::dec;
    }
    ostream << ecl::reset;
    std::cout << ostream.str() << std::endl;
  }

private:
  kobuki::Kobuki kobuki;
  ecl::Slot<kobuki::PacketFinder::BufferType&> slot_raw_data_stream;
};

/*****************************************************************************
 ** Main
 *****************************************************************************/

int main(int argc, char **argv)
{
  ecl::CmdLine cmd_line("raw_data_stream", ' ', "0.3");
  ecl::ValueArg<std::string> device_port(
      "p", "port",
      "Path to device file of serial port to open",
      false,
      "/dev/kobuki",
      "string"
  );
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);

  std::cout << ecl::bold << "\nRaw Data Stream Demo\n" << ecl::reset << std::endl;

  KobukiManager kobuki_manager(device_port.getValue());
  ecl::Sleep()(5);
  return 0;
}
