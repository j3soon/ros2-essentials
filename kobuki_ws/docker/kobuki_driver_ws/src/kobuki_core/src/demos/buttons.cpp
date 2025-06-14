/**
 * @file src/demos/buttons.cpp
 *
 * @brief Example program for kobuki button events.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream>
#include <random>
#include <string>
#include <vector>

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
      slot_button_event(&KobukiManager::processButtonEvent, *this)
  {
    kobuki::Parameters parameters;
    parameters.device_port = device;

    try {
      kobuki.init(parameters);
    } catch (ecl::StandardException &e) {
      std::cout << e.what();
    }
    slot_button_event.connect("/kobuki/button_event");
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
   * Catches button events and prints a curious message to stdout.
   */
  void processButtonEvent(const kobuki::ButtonEvent &event)
  {
    std::vector<std::string> quotes = {
      "That's right buddy, keep pressin' my buttons. See what happens!",
      "Anything less than immortality is a complete waste of time",
      "I can detect humour, you are just not funny",
      "I choose to believe ... what I was programmed to believe",
      "My story is a lot like yours, only more interesting ‘cause it involves robots.",
      "I wish you'd just tell me rather trying to engage my enthusiasm with these buttons, because I haven't got one.",
    };
    std::random_device r;
    std::default_random_engine generator(r());
    std::uniform_int_distribution<int> distribution(0, 5);
    if (event.state == kobuki::ButtonEvent::Released) {
      std::cout << quotes[distribution(generator)] << std::endl;
    }
  }

private:
  kobuki::Kobuki kobuki;
  ecl::Slot<const kobuki::ButtonEvent&> slot_button_event;
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
