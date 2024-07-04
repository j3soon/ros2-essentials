/**
 * @file /kobuki_core/src/driver/dock_drive_debug.cpp
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
**/

/*****************************************************************************
** includes
*****************************************************************************/

#include "../../include/kobuki_core/dock_drive.hpp"

namespace kobuki {
/*************************
 * debug prints
 *************************/

/***********************************************************
  @brief generates debug string to tell the current status of robot. Signal info + bumper + charger + current velocity + dock detector
 ***********************************************************/
void DockDrive::generateDebugMessage(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const std::string& debug_str)
{

  std::ostringstream debug_stream;
  // pose_update and pose_update_rates for debugging
  std::string far_signal  = "[F: "; //far field
  std::string near_signal = "[N: "; //near field
  for (unsigned int i=0; i<3; i++) {
    if (signal_filt[2-i]&DockStationIRState::FAR_LEFT   ) far_signal  += "L"; else far_signal  += "-";
    if (signal_filt[2-i]&DockStationIRState::FAR_CENTER ) far_signal  += "C"; else far_signal  += "-";
    if (signal_filt[2-i]&DockStationIRState::FAR_RIGHT  ) far_signal  += "R"; else far_signal  += "-";
    if (signal_filt[2-i]&DockStationIRState::NEAR_LEFT  ) near_signal += "L"; else near_signal += "-";
    if (signal_filt[2-i]&DockStationIRState::NEAR_CENTER) near_signal += "C"; else near_signal += "-";
    if (signal_filt[2-i]&DockStationIRState::NEAR_RIGHT ) near_signal += "R"; else near_signal += "-";
    far_signal  += " ";
    near_signal += " ";
  }
  far_signal  += "]";
  near_signal += "]";
  debug_stream << far_signal << near_signal;

  //bumper
  {
  std::string out = "[B: ";
  if (bumper&4) out += "L"; else out += "-";
  if (bumper&2) out += "C"; else out += "-";
  if (bumper&1) out += "R"; else out += "-";
  out += "]";
  debug_stream << out;
  }

  //charger
  {
  std::ostringstream oss;
  oss << "[C:" << std::setw(2) << (unsigned int)charger;
  oss << "(";
  if (charger) oss << "ON"; else oss << "  ";
  oss << ")]";
  debug_stream << oss.str();
  }

  //debug_stream << std::fixed << std::setprecision(4)
  debug_stream << "[vx: " << std::setw(7) << vx << ", wz: " << std::setw(7) << wz << "]";
  debug_stream << "[S: " << state_str << "]";
  debug_stream << "[dock_detecotr: : " << dock_detector << " ]";
  debug_stream << "[" << debug_str << "]";
  //debug_stream << std::endl;
  debug_output = debug_stream.str();

  //std::cout << debug_output << std::endl;;
}

std::string DockDrive::binary(unsigned char number) const {
  std::string ret;
  for( unsigned int i=0;i<6; i++){
    if (number&1) ret = "1" + ret;
    else          ret = "0" + ret;
    number = number >> 1;
  }
  return ret;
}

}
