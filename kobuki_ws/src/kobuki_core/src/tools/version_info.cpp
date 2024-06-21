/**                                                                           !
 * @file /kobuki_core/src/tools/version_info.cpp
 *
 * @brief Tools/utility program to retriving version info. of kobuki.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include <functional>
#include <string>

#include <ecl/console.hpp>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/command_line.hpp>

#include "kobuki_core/kobuki.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

class KobukiManager {
public:
  KobukiManager(const std::string &device_port) :
    acquired(false),
    slot_version_info(&KobukiManager::processVersionInfo, *this)
  {
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/kobuki"; // configure the first part of the sigslot namespace
    parameters.device_port = device_port;    // the serial port to connect to (windows COM1..)
    kobuki.init(parameters);
    slot_version_info.connect(parameters.sigslots_namespace + "/version_info");
  }

  ~KobukiManager() {}

  void processVersionInfo(const kobuki::VersionInfo &version_info) {
    hardware = kobuki::VersionInfo::toString(version_info.hardware);
    firmware = kobuki::VersionInfo::toString(version_info.firmware);
    //software = kobuki::VersionInfo::toString(version_info.software);
    software = kobuki::VersionInfo::getSoftwareVersion();
    udid = kobuki::VersionInfo::toString(version_info.udid0, version_info.udid1, version_info.udid2);
    acquired = true;
  }

  bool isAcquired() { return acquired; }
  std::string& getHardwareVersion() { return hardware; }
  std::string& getFirmwareVersion() { return firmware; }
  std::string& getSoftwareVersion() { return software; }
  std::string& getUDID() { return udid; }

private:
  volatile bool acquired;
  kobuki::Kobuki kobuki;
  std::string hardware, firmware, software, udid;
  ecl::Slot<const kobuki::VersionInfo&> slot_version_info;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  ecl::CmdLine cmd_line("version_info program", ' ', "0.3");
  ecl::UnlabeledValueArg<std::string> device_port("device_port", "Path to device file of serial port to open, connected to the kobuki", false, "/dev/kobuki", "string");
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);
  //std::cout << "device_port: " << device_port.getValue() << std::endl;

  std::cout << ecl::bold << "\nVersion Info\n" << ecl::reset << std::endl;
  KobukiManager kobuki_manager(device_port.getValue());

  ecl::MilliSleep sleep_one_hundred_ms(100);
  while (!kobuki_manager.isAcquired()) { sleep_one_hundred_ms(); }
  std::cout << " * Hardware Version: " << kobuki_manager.getHardwareVersion() << std::endl;
  std::cout << " * Firmware Version: " << kobuki_manager.getFirmwareVersion() << std::endl;
  std::cout << " * Software Version: " << kobuki_manager.getSoftwareVersion() << std::endl;
  std::cout << " * Unique Device ID: " << kobuki_manager.getUDID() << std::endl;

  return 0;
}
