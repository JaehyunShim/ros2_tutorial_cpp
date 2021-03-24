// Copyright (c) 2018 Intel Corporation
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

#ifndef GDB_TUTORIAL_CPP__GDB_HPP_
#define GDB_TUTORIAL_CPP__GDB_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace gdb_tutorial_cpp
{

// Borrowed crash examples from (https://github.com/samsung-ros/gdb_test_pkg)
class PtrClass
{
public:
  double data;
};

class GDB : public rclcpp::Node
{
public:
  GDB()
  : Node("gdb")
  {

  // Borrowed crash examples from (https://github.com/samsung-ros/gdb_test_pkg)
    // exit_crash();
    nullptr_crash();
    // vector_crash();
  }

  void exit_crash()
  {
    RCLCPP_INFO(this->get_logger(), "Exit Crashing...");
    exit(-1);
  }

  void nullptr_crash()
  {
    RCLCPP_INFO(this->get_logger(), "Nullptr Crashing...");
    PtrClass * ptr = nullptr;

    // Uncomment the following line to see gdb find carshing line.
    // RCLCPP_INFO(this->get_logger(), "%f...", ptr->data);
  }

  void vector_crash()
  {
    RCLCPP_INFO(this->get_logger(), "Vector Crashing...");
    std::vector<int> vec;
    vec.at(100) = 5;
  }
};
}  // namespace gdb_tutorial_cpp
#endif  // GDB_TUTORIAL_CPP__GDB_HPP_
