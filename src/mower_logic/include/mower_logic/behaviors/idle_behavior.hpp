// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
// SPDX-License-Identifier: GPL-3.0
//
// ROS2 port of IdleBehavior

#ifndef MOWER_LOGIC__BEHAVIORS__IDLE_BEHAVIOR_HPP_
#define MOWER_LOGIC__BEHAVIORS__IDLE_BEHAVIOR_HPP_

#include <vector>
#include "mower_logic/behaviors/behavior.hpp"
#include "xbot_msgs/msg/action_info.hpp"

namespace mower_logic
{

// Forward declarations
class MowingBehavior;
class UndockingBehavior;
class AreaRecordingBehavior;

class IdleBehavior : public Behavior
{
public:
  explicit IdleBehavior(bool stay_docked = false);

  static IdleBehavior INSTANCE;
  static IdleBehavior DOCKED_INSTANCE;

  std::string state_name() override;
  Behavior* execute() override;
  void enter() override;
  void exit() override;
  void reset() override;
  bool needs_gps() override;
  bool mower_enabled() override;
  bool redirect_joystick() override;
  void command_home() override;
  void command_start() override;
  void command_s1() override;
  void command_s2() override;
  uint8_t get_sub_state() override;
  uint8_t get_state() override;
  void handle_action(const std::string& action) override;

private:
  bool stay_docked_ = false;
  bool manual_start_mowing_ = false;
  bool start_area_recorder_ = false;
  std::vector<xbot_msgs::msg::ActionInfo> actions_;
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__BEHAVIORS__IDLE_BEHAVIOR_HPP_
