// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef INTERBOTIX_XS_RVIZ__XS_REGISTER_DESCRIPTIONS_HPP_
#define INTERBOTIX_XS_RVIZ__XS_REGISTER_DESCRIPTIONS_HPP_

#include <string>
#include <unordered_map>

namespace xs_register_descriptions
{

typedef struct
{
  std::string name;
  std::string description;
  int address;
  std::string units;
} XSRegisterDescription;

static XSRegisterDescription desc_operating_mode = {
  "Operating_Mode",
  "See the OperatingModes service definition for details.",
  11,
  "-"
};

static XSRegisterDescription desc_profile_velocity = {
  "Profile_Velocity",
  "When the Operating Mode is Velocity-based Profile, Profile Velocity sets the maximum velocity "
  "of the Profile. When the Operating Mode is Time-based Profile, Profile Velocity sets the time "
  "span to reach the velocity (the total time) of the Profile. Be aware that the Profile Velocity "
  "is to be only applied to Position Control Mode, Extended Position Control Mode or Current-based"
  " Position Control Mode on the Operating Mode.",
  112,
  "0.229 [rev/min]"
};

static XSRegisterDescription desc_profile_acceleration = {
  "Profile_Acceleration",
  "When the Operating Mode is Velocity-based Profile, Profile Acceleration sets acceleration of "
  "the Profile. When the Operating Mode is Time-based Profile, Profile Acceleration sets "
  "acceleration time of the Profile. The Profile Acceleration is to be applied in all control mode"
  " except Current Control Mode or PWM Control Mode on the Operating Mode.",
  108,
  "214.577 [rev/min2]"
};

static XSRegisterDescription desc_goal_position = {
  "Goal_Position",
  "The Goal Position sets desired position [pulses]. From the front view of DYNAMIXEL, CCW is an "
  "increasing direction, whereas CW is a decreasing direction. Units: 1 pulse",
  116,
  "pulses"
};

static XSRegisterDescription desc_goal_velocity = {
  "Goal_Velocity",
  "Use the Goal Velocity to set a desired velocity when the Operating Mode is Velocity Control "
  "Mode. Units: 0.229 rev/min",
  104,
  "0.229 rev/min"
};

static XSRegisterDescription desc_goal_current = {
  "Goal_Current",
  "Use Goal Current to set a desired current when the Operating Mode is Torque Control Mode. The "
  "Goal Current can also be used to set current limits in Current-based Position Control Mode. "
  "Units: 2.69 mA",
  102,
  "2.69 mA"
};

static XSRegisterDescription desc_goal_pwm = {
  "Goal_PWM",
  "When the Operating Mode is PWM Control Mode, both the PID and Feedforward controllers will be "
  "deactivated as the Goal PWM value directly controls a motor via an inverter. But on the other "
  "Operating Mode, the Goal PWM limits PWM value only. Units: 0.113%",
  100,
  "0.113%"
};

static XSRegisterDescription desc_present_position = {
  "Present_Position",
  "The Present Position indicates present Position. Units: 1 pulse",
  132,
  "1 pulse"
};

static XSRegisterDescription desc_present_velocity = {
  "Present_Velocity",
  "This value indicates current Velocity. Units: 0.229 rev/min",
  128,
  "0.229 rev/min"
};

static XSRegisterDescription desc_present_current = {
  "Present_Current",
  "This value indicates current Current. Units: 2.69mA",
  126,
  "2.69 mA"
};

static XSRegisterDescription desc_present_temperature = {
  "Present_Temperature",
  "The Present Temperature indicates internal temperature of DYNAMIXEL. Units: 1 degree Celsius",
  146,
  "1 degree Celsius"
};

static std::unordered_map<std::string, XSRegisterDescription> descriptions = {
  {desc_operating_mode.name, desc_operating_mode},
  {desc_profile_velocity.name, desc_profile_velocity},
  {desc_profile_acceleration.name, desc_profile_acceleration},
  {desc_goal_position.name, desc_goal_position},
  {desc_goal_velocity.name, desc_goal_velocity},
  {desc_goal_current.name, desc_goal_current},
  {desc_goal_pwm.name, desc_goal_pwm},
  {desc_present_position.name, desc_present_position},
  {desc_present_velocity.name, desc_present_velocity},
  {desc_present_current.name, desc_present_current},
  {desc_present_temperature.name, desc_present_temperature}
};

}  // namespace xs_register_descriptions

#endif  // INTERBOTIX_XS_RVIZ__XS_REGISTER_DESCRIPTIONS_HPP_
