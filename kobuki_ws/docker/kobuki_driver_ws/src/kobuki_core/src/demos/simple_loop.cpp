/**                                                                           !
 * @file src/demos/simple_loop.cpp
 *
 * @brief Example program with a simple control loop.
 *
 * Controls the kobuki around a dead-reckoned square with sides of
 * length 0.25m.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include <string>

#include <csignal>
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

#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/command_line.hpp>
#include "kobuki_core/kobuki.hpp"


/*****************************************************************************
** Classes
*****************************************************************************/

class KobukiManager {
public:
  KobukiManager(
      const std::string & device,
      const double &length,
      const bool &disable_smoothing
  ) :
    dx(0.0), dth(0.0),
    length(length),
    slot_stream_data(&KobukiManager::processStreamData, *this)
  {
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/kobuki";
    parameters.device_port = device;
    parameters.enable_acceleration_limiter = !disable_smoothing;

    kobuki.init(parameters);
    kobuki.enable();
    slot_stream_data.connect("/kobuki/stream_data");
  }

  ~KobukiManager()
  {
    kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
    kobuki.disable();
  }

  void processStreamData()
  {
    ecl::linear_algebra::Vector3d pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    ecl::concatenate_poses(pose, pose_update);
    dx += pose_update[0];   // x
    dth += pose_update[2];  // heading
    // std::cout << dx << ", " << dth << std::endl;
    // std::cout << kobuki.getHeading() << ", " << pose.heading() << std::endl;
    // std::cout << "[" << pose[0] << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    processMotion();
  }

  // Generate square motion
  void processMotion()
  {
    const double buffer = 0.05;
    double longitudinal_velocity = 0.0;
    double rotational_velocity = 0.0;
    if (dx >= (length) && dth >= ecl::pi/2.0) {
      std::cout << "[Z] ";
      dx=0.0; dth=0.0;
    } else if (dx >= (length + buffer)) {
      std::cout << "[R] ";
      rotational_velocity = 1.1;
    } else {
      std::cout << "[L] ";
      longitudinal_velocity = 0.3;
    }
    std::cout << "[dx: " << dx << "][dth: " << dth << "][" << pose[0] << ", " << pose[1] << ", " << pose[2] << "]" << std::endl;
    kobuki.setBaseControl(longitudinal_velocity, rotational_velocity);
  }

  const ecl::linear_algebra::Vector3d& getPose()
  {
    return pose;
  }

private:
  double dx, dth;
  const double length;
  ecl::linear_algebra::Vector3d pose;  // x, y, heading
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
};

/*****************************************************************************
** Signal Handler
*****************************************************************************/

bool shutdown_req = false;
void signalHandler(int /* signum */)
{
  shutdown_req = true;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  ecl::CmdLine cmd_line("Uses a simple control loop to move Kobuki around a dead-reckoned square with sides of length 1.0m", ' ', "0.2");
  ecl::ValueArg<std::string> device_port(
      "p", "port",
      "Path to device file of serial port to open",
      false,
      "/dev/kobuki",
      "string"
  );
  ecl::ValueArg<double> length(
      "l", "length",
      "traverse square with sides of this size in length (m)",
      false,
      0.25,
      "double"
  );
  ecl::SwitchArg disable_smoothing(
      "d", "disable_smoothing",
      "Disable the acceleration limiter (smoothens velocity)",
      false
  );

  cmd_line.add(device_port);
  cmd_line.add(length);
  cmd_line.add(disable_smoothing);
  cmd_line.parse(argc, argv);

  signal(SIGINT, signalHandler);

  std::cout << "Demo : Example of simple control loop." << std::endl;
  KobukiManager kobuki_manager(
      device_port.getValue(),
      length.getValue(),
      disable_smoothing.getValue()
  );

  ecl::Sleep sleep(1);
  ecl::linear_algebra::Vector3d pose;  // x, y, heading
  try {
    while (!shutdown_req){
      sleep();
      pose = kobuki_manager.getPose();
      // std::cout << "current pose: [" << pose[0] << ", " << pose[1] << ", " << pose[2] << "]" << std::endl;
    }
  } catch (ecl::StandardException &e) {
    std::cout << e.what();
  }
  return 0;
}
