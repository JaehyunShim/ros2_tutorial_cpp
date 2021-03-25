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

#ifndef SETUP_ASSISTANT_TUTORIAL_CPP__SETUP_ASSISTANT_TUTORIAL_HPP_
#define SETUP_ASSISTANT_TUTORIAL_CPP__SETUP_ASSISTANT_TUTORIAL_HPP_

#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

namespace setup_assistant_tutorial
{
std::array<std::pair<std::string, std::string>, 4> str_pair_arr = {
  std::make_pair("{package_name}", "test_config"),
  std::make_pair("{author_name}", "Jaehyun Shim"),
  std::make_pair("{author_email_address}", "jhshim@robotis.com"),
  std::make_pair("{robot_name}", "Jaehyunbot")};

std::vector<std::string> read_file(std::string file_name)
{
  std::vector<std::string> read_lines;
  std::string read_line;
  std::ifstream MyReadFile(file_name.c_str());
  while (getline(MyReadFile, read_line)) {
    read_lines.push_back(read_line);
    // std::cout << read_line << std::endl;
  }
  MyReadFile.close();
  return read_lines;
}

std::string replace_line(std::string line, std::pair<std::string, std::string> pair)
{
  std::string::size_type offset = 0;
  offset = line.find(pair.first, offset);
  if (offset != std::string::npos) {
    line.replace(offset, pair.first.length(), pair.second);
  }
  return line;
}

void write_file(std::string file_name, std::vector<std::string> write_lines)
{
  std::ofstream MyWriteFile(file_name.c_str());
  for (std::string & line : write_lines) {
    for (uint8_t i = 0; i < str_pair_arr.size(); i++) {
      line = replace_line(line, str_pair_arr[i]);
    }
    MyWriteFile << line << std::endl;
  }
  MyWriteFile.close();
}

void generate_file(std::string file_name, std::string file_name2)
{
  std::vector<std::string> read_lines;

  std::string read_file_name(file_name);
  std::string read_file_path = read_file_name;
  std::string write_file_name(file_name2);
  std::string write_file_path = write_file_name;

  // std::cout << read_file_path << std::endl;
  // std::cout << write_file_path << std::endl;

  read_lines = read_file(read_file_path);
  write_file(write_file_path, read_lines);
}

void generate_directory(std::string dir_name)
{
  boost::filesystem::path dstFolder = dir_name;
  boost::filesystem::create_directory(dstFolder);
}

class SetupAssistantTutorial : public rclcpp::Node
{
public:
  SetupAssistantTutorial()
  : Node("setup_assistant_tutorial")
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("setup_assistant_tutorial_cpp");

    std::string new_package_path("/home/robotis/colcon_ws/src/test_config");

    // Check if the directory already exists
    if (boost::filesystem::exists(new_package_path)) {
      std::cout << "Already exists" << std::endl;
      rclcpp::shutdown();
    } else {
      std::cout << "Create" << std::endl;

      generate_directory(new_package_path);
      generate_directory(new_package_path + "/config");

      generate_file(
        package_share_directory + "/templates/CMakeLists.txt.template",
        new_package_path + "/CMakeLists.txt");
      generate_file(
        package_share_directory + "/templates/package.xml.template",
        new_package_path + "/package.xml");
      generate_file(
        package_share_directory + "/templates/config/test.yaml",
        new_package_path + "/config/test.yaml");
    }
  }
};
}  // namespace setup_assistant_tutorial
#endif  // SETUP_ASSISTANT_TUTORIAL_CPP__SETUP_ASSISTANT_TUTORIAL_HPP_
