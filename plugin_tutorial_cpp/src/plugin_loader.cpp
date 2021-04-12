// Copyright (c) 2012, Willow Garage, Inc.
// Copyright 2021 Jaehyun Shim
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

#include <signal.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "plugin_tutorial_cpp/plugin_base.hpp"

#define UNUSED(x) (void)(x)

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

static bool active = true;
void active_handler(int dummy)
{
  UNUSED(dummy);
  active = false;
}

int main(int argc, char ** argv)
{
  // init
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // plugin
  pluginlib::ClassLoader<plugin_tutorial_cpp::PluginBase> class_loader_(
    "plugin_tutorial_cpp",
    "plugin_tutorial_cpp::PluginBase");
  std::shared_ptr<plugin_tutorial_cpp::PluginBase> plugin_class_ = nullptr;
  for (const auto & c : class_loader_.getDeclaredClasses()) {
    printf("%s\n", c.c_str());
  }
  plugin_class_ = class_loader_.createSharedInstance("plugin_tutorial_cpp/Plugin");

  // ros executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(plugin_class_);

  // spin
  rclcpp::Rate loop_rate(10);
  signal(SIGINT, active_handler);
  while (active) {
    // update and spin_some
    plugin_class_->update();
    executor.spin_some();

    // sleep
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
