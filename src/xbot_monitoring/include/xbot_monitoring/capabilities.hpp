// Copyright (c) 2022 Clemens Elflein. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS2 port of xbot_monitoring capabilities

#pragma once

#include <nlohmann/json.hpp>

namespace xbot_monitoring
{

inline const nlohmann::ordered_json CAPABILITIES = {
  { "rpc", 1 },
  { "map:json", 1 },
};

}  // namespace xbot_monitoring
