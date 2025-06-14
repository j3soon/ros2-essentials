/**
 * @file /kobuki_core/src/tools/simple_keyop.cpp
 *
 * @brief Tools/utility program to control robot by keyboard.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <csignal>
#include <termios.h> // for keyboard input

#include <ecl/command_line.hpp>
#include <ecl/console.hpp>
// Version 3.4.0 of Eigen in Ubuntu 22.04 has a bug that causes -Wclass-memaccess warnings on
// aarch64.  Upstream Eigen has already fixed this in
// https://gitlab.com/libeigen/eigen/-/merge_requests/645 .  The Debian fix for this is in
// https://salsa.debian.org/science-team/eigen3/-/merge_requests/1 .
// However, it is not clear that that fix is going to make it into Ubuntu 22.04 before it
// freezes, so disable the warning here.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#endif
#include <ecl/geometry.hpp>
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <ecl/linear_algebra.hpp>
#include <ecl/time.hpp>
#include <ecl/threads.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/exceptions.hpp>

#include "kobuki_core/kobuki.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

/**
 * @brief Keyboard remote control for our robot core (mobile base).
 *
 */
class KobukiManager
{
public:

  /*********************
   ** C&D
   **********************/
  KobukiManager();
  ~KobukiManager();
  bool init(const std::string & device);

  /*********************
   ** Callbacks
   **********************/
  void processStreamData();

  /*********************
   ** Accessor
   **********************/
  const ecl::linear_algebra::Vector3d& getPose() { return pose; };
  bool isShutdown() { return quit_requested || kobuki.isShutdown(); }

private:
  double vx, wz;
  ecl::linear_algebra::Vector3d pose;
  kobuki::Kobuki kobuki;

  double linear_vel_step, linear_vel_max;
  double angular_vel_step, angular_vel_max;
  std::string name;
  ecl::Slot<> slot_stream_data;

  /*********************
   ** Commands
   **********************/
  void incrementLinearVelocity();
  void decrementLinearVelocity();
  void incrementAngularVelocity();
  void decrementAngularVelocity();
  void resetVelocity();

  /*********************
   ** Debugging
   **********************/
  void relayWarnings(const std::string& message);
  void relayErrors(const std::string& message);

  /*********************
   ** Keylogging
   **********************/
  void keyboardInputLoop();
  void processKeyboardInput(char c);
  void restoreTerminal();
  bool quit_requested;
  int key_file_descriptor;
  struct termios original_terminal_state;
  ecl::Thread thread;
  ecl::Mutex mutex;
};

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

/**
 * @brief Default constructor, needs initialisation.
 */
KobukiManager::KobukiManager() :
  vx(0.0), wz(0.0),
  linear_vel_step(0.05),
  linear_vel_max(1.0),
  angular_vel_step(0.33),
  angular_vel_max(6.6),
  slot_stream_data(&KobukiManager::processStreamData, *this),
  quit_requested(false),
  key_file_descriptor(0)
{
  tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

KobukiManager::~KobukiManager()
{
  kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
  kobuki.disable();
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
 * @brief Initialises the node.
 */
bool KobukiManager::init(const std::string & device)
{
  /*********************
   ** Parameters
   **********************/
  std::cout << "Parameters" << std::endl;
  std::cout << "----------" << std::endl;
  std::cout << " - linear_vel_max   [" << linear_vel_max << "]" << std::endl;
  std::cout << " - linear_vel_step  [" << linear_vel_step << "]" << std::endl;
  std::cout << " - angular_vel_max  [" << angular_vel_max << "]" << std::endl;
  std::cout << " - angular_vel_step [" << angular_vel_step << "]" << std::endl;
  std::cout << std::endl;

  /*********************
   ** Velocities
   **********************/
  vx = 0.0;
  wz = 0.0;

  /*********************
   ** Kobuki
   **********************/
  kobuki::Parameters parameters;
  parameters.sigslots_namespace = "/kobuki";
  parameters.device_port = device;
  parameters.enable_acceleration_limiter = true;

  kobuki.init(parameters);
  kobuki.enable();
  slot_stream_data.connect("/kobuki/stream_data");

  /*********************
   ** Wait for connection
   **********************/
  thread.start(&KobukiManager::keyboardInputLoop, *this);
  return true;
}

/*****************************************************************************
 ** Implementation [Keyboard]
 *****************************************************************************/

/**
 * @brief The worker thread function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */
void KobukiManager::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor, TCSANOW, &raw);

  std::cout << "Reading from keyboard" << std::endl;
  std::cout << "---------------------" << std::endl;
  std::cout << "Forward/back arrows : linear velocity incr/decr." << std::endl;
  std::cout << "Right/left arrows : angular velocity incr/decr." << std::endl;
  std::cout << "Spacebar : reset linear/angular velocities." << std::endl;
  std::cout << "q : quit." << std::endl;
  std::cout << std::endl;
  char c;
  while (!quit_requested)
  {
    if (read(key_file_descriptor, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    processKeyboardInput(c);
  }
}

/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void KobukiManager::processKeyboardInput(char c)
{
  /*
   * Arrow keys are a bit special, they are escape characters - meaning they
   * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
   * ignore the esc-[ and just parse the last one. So long as we avoid using
   * the last one for its actual purpose (e.g. left arrow corresponds to
   * esc-[-D) we can keep the parsing simple.
   */
  switch (c)
  {
    case 68://kobuki_msgs::KeyboardInput::KEYCODE_LEFT:
    {
      incrementAngularVelocity();
      break;
    }
    case 67://kobuki_msgs::KeyboardInput::KEYCODE_RIGHT:
    {
      decrementAngularVelocity();
      break;
    }
    case 65://kobuki_msgs::KeyboardInput::KEYCODE_UP:
    {
      incrementLinearVelocity();
      break;
    }
    case 66://kobuki_msgs::KeyboardInput::KEYCODE_DOWN:
    {
      decrementLinearVelocity();
      break;
    }
    case 32://kobuki_msgs::KeyboardInput::KEYCODE_SPACE:
    {
      resetVelocity();
      break;
    }
    case 'q':
    {
      quit_requested = true;
      break;
    }
    default:
    {
      break;
    }
  }
}

/*****************************************************************************
 ** Implementation [Commands]
 *****************************************************************************/

/**
 * @brief If not already maxxed, increment the command velocities..
 */
void KobukiManager::incrementLinearVelocity()
{
  mutex.lock();
  if (vx <= linear_vel_max)
  {
    vx += linear_vel_step;
  }
  mutex.unlock();
}

/**
 * @brief If not already minned, decrement the linear velocities..
 */
void KobukiManager::decrementLinearVelocity()
{
  mutex.lock();
  if (vx >= -linear_vel_max)
  {
    vx -= linear_vel_step;
  }
  mutex.unlock();
}

/**
 * @brief If not already maxxed, increment the angular velocities..
 */
void KobukiManager::incrementAngularVelocity()
{
  mutex.lock();
  if (wz <= angular_vel_max)
  {
    wz += angular_vel_step;
  }
  mutex.unlock();
}

/**
 * @brief If not already mined, decrement the angular velocities..
 */
void KobukiManager::decrementAngularVelocity()
{
  mutex.lock();
  if (wz >= -angular_vel_max)
  {
    wz -= angular_vel_step;
  }
  mutex.unlock();
}

void KobukiManager::resetVelocity()
{
  mutex.lock();
  vx = 0.0;
  wz = 0.0;
  mutex.unlock();
}

void KobukiManager::processStreamData() {
  ecl::linear_algebra::Vector3d pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kobuki.updateOdometry(pose_update, pose_update_rates);
  ecl::concatenate_poses(pose, pose_update);
  // TODO(daniel.stonier): this needs a mutex
  // This callback triggers in Kobuki's thread, however
  // vx, wz are updated in the keyboard input thread.
  mutex.lock();
  kobuki.setBaseControl(vx, wz);
  mutex.unlock();
}

/*****************************************************************************
** Signal Handler
*****************************************************************************/

bool signal_shutdown_requested = false;
void signalHandler(int /* signum */) {
  signal_shutdown_requested = true;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  ecl::CmdLine cmd_line("simple_keyop program", ' ', "0.3");
  ecl::UnlabeledValueArg<std::string> device_port("device_port", "Path to device file of serial port to open, connected to the kobuki", false, "/dev/kobuki", "string");
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);

  signal(SIGINT, signalHandler);

  std::cout << ecl::bold << "\nSimple Keyop : Utility for driving kobuki by keyboard.\n" << ecl::reset << std::endl;
  KobukiManager kobuki_manager;
  kobuki_manager.init(device_port.getValue());

  ecl::Sleep sleep_one_second(1);
  try {
    while (!signal_shutdown_requested && !kobuki_manager.isShutdown()){
      sleep_one_second();
      const ecl::linear_algebra::Vector3d& pose = kobuki_manager.getPose();
      std::cout << ecl::green;
      std::cout << "current pose: [x: " << pose[0] << ", y: " << pose[1] << ", heading: " << pose[2] << "]" << std::endl;
      std::cout << ecl::reset;
    }
  } catch ( ecl::StandardException &e ) {
    std::cout << e.what();
  }
  return 0;
}
