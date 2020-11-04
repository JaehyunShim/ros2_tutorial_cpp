// Copyright (c) 2012, Willow Garage, Inc.
// Copyright 2020, Jaehyun Shim, ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLUGIN_EXAMPLE__PLUGIN_BASE_HPP_
#define PLUGIN_EXAMPLE__PLUGIN_BASE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace plugin_example
{
class PluginBase : public rclcpp::Node
{
public:
  virtual void update() = 0;
  virtual ~PluginBase() {}

protected:
  PluginBase()
  : Node("plugin_base") {}
};
}  // namespace plugin_example

#endif  // PLUGIN_EXAMPLE__PLUGIN_BASE_HPP_
