/**
 * @file /kobuki_dock_drive/include/kobuki_dock_drive/dock_drive.hpp
 *
 * @brief Simple module for the docking drive algorithm
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
**/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_CORE_DOCK_DRIVE_HPP_
#define KOBUKI_CORE_DOCK_DRIVE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <ecl/geometry.hpp>
#include <ecl/linear_algebra.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Definitions
*****************************************************************************/

// indicates the ir sensor from docking station
struct DockStationIRState {
  enum State {
    INVISIBLE=0,
    NEAR_LEFT=1,
    NEAR_CENTER=2,
    NEAR_RIGHT=4,
    FAR_CENTER=8,
    FAR_LEFT=16,
    FAR_RIGHT=32,
    NEAR = 7, // NEAR_LEFT + NEAR_CENTER + NEAR_RIGHT
    FAR = 56, // FAR_CENTER + FAR_LEFT + FAR_RIGHT
  };
};

// the current robot states
struct RobotDockingState {
  enum State {
    IDLE,
    DONE,
    DOCKED_IN,
    BUMPED_DOCK,
    BUMPED,
    SCAN,
    FIND_STREAM,
    GET_STREAM,
    ALIGNED,
    ALIGNED_FAR,
    ALIGNED_NEAR,
    UNKNOWN,
    LOST
  };

};

/*****************************************************************************
** Interfaces
*****************************************************************************/

class DockDrive {
public:
  DockDrive();
  ~DockDrive();

  bool init(){ return true; }
  bool isEnabled() const { return is_enabled; }
  bool canRun() const { return can_run; }

  void enable() { modeShift("enable"); }
  void disable() { modeShift("disable"); }
  void modeShift(const std::string& mode);

  void update(const std::vector<unsigned char> &signal /* dock_ir signal*/
                , const unsigned char &bumper
                , const unsigned char &charger
                , const ecl::linear_algebra::Vector3d &pose);

  void velocityCommands(const double &vx, const double &wz);

  /*********************
  ** Command Accessors
  **********************/
  double getVX() const { return vx; }
  double getWZ() const { return wz; }

  /*********************
  ** Mode Accessors
  **********************/
  RobotDockingState::State getState() const { return state; }
  std::string getStateStr() const { return state_str; }
  std::string getDebugStr() const { return debug_str; }

  /*********************
  ** Parameters Mutators
  **********************/
  void setMinAbsV(double mav) { min_abs_v = mav; }
  void setMinAbsW(double maw) { min_abs_w = maw; }

  //debugging
  std::string getDebugStream() { return debug_output; } //stream.str(); }
  //std::string getDebugStream() { return debug_stream.str(); }
  //std::ostringstream debug_stream;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void processBumpChargeEvent(const unsigned char& bumper, const unsigned char& charger);
  void computePoseUpdate(ecl::linear_algebra::Vector3d& pose_update, const ecl::linear_algebra::Vector3d& pose);
  void filterIRSensor(std::vector<unsigned char>& signal_filt,const std::vector<unsigned char> &signal );
  void generateDebugMessage(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const std::string& debug_str);
  void updateVelocity(const std::vector<unsigned char>& signal_filt, const ecl::linear_algebra::Vector3d& pose_update, std::string& debug_str);
  RobotDockingState::State determineRobotLocation(const std::vector<unsigned char>& signal_filt,const unsigned char& charger);
  bool validateSignal(const std::vector<unsigned char>& signal_filt, const unsigned int state);


  // States
  void idle(RobotDockingState::State& state,double& vx, double& wz);
  void scan(RobotDockingState::State& state,double& vx, double& wz, const std::vector<unsigned char>& signal_filt, const ecl::linear_algebra::Vector3d& pose_update, std::string& debug_str);
  void find_stream(RobotDockingState::State& state,double& vx, double& wz, const std::vector<unsigned char>& signal_filt);
  void get_stream(RobotDockingState::State& state,double& vx, double& wz, const std::vector<unsigned char>& signal_filt);
  void aligned(RobotDockingState::State& state,double& vx, double& wz, const std::vector<unsigned char>& signal_filt, std::string& debug_str);
  void bumped(RobotDockingState::State& nstate,double& nvx, double& nwz, int& bump_count);


private:
  bool is_enabled, can_run;

  RobotDockingState::State state;
  std::string state_str, debug_str;
  double vx, wz;
  std::vector<std::vector<unsigned char> > past_signals;
  unsigned int signal_window;
  int bump_remainder;
  int dock_stabilizer;
  int dock_detector;
  double rotated;
  double min_abs_v;
  double min_abs_w;
  ecl::linear_algebra::Vector3d pose_priv;

  void setVel(double v, double w);

  std::string binary(unsigned char number) const;

  std::string debug_output;
  std::vector<std::string> ROBOT_STATE_STR;
};

} // namespace kobuki

#endif /* KOBUKI_CORE_DOCK_DRIVE_HPP_ */
