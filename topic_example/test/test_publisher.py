# # Copyright 2019 Open Source Robotics Foundation, Inc.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
# #
# # Authors: Ryan Shim

# import os

# import unittest

# from launch import LaunchDescription
# from launch.actions import ExecuteProcess

# import launch_testing
# import launch_testing.actions
# import launch_testing.asserts
# import launch_testing.util
# import launch_testing_ros


# def generate_test_description():
#     env = os.environ.copy()
#     env['OSPL_VERBOSITY'] = '8'  # 8 = OS_NONE
#     # bare minimum formatting for console output matching
#     env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

#     launch_description = LaunchDescription()
#     processes_under_test = [
#         ExecuteProcess(
#             cmd=[executable],
#             name='test_executable_' + str(i),
#             output='screen',
#             env=env)
#         for i, executable in enumerate('@DEMO_NODES_CPP_EXECUTABLE@'.split(';'))
#     ]
#     for process in processes_under_test:
#         launch_description.add_action(process)
#     launch_description.add_action(launch_testing.util.KeepAliveProc())
#     launch_description.add_action(
#         launch_testing.actions.ReadyToTest()
#     )
#     return launch_description, locals()


# class TestExecutablesTutorial(unittest.TestCase):

#     def test_processes_output(self, proc_output, processes_under_test):
#         """Test all processes output against expectations."""
#         from launch_testing.tools.output import get_default_filtered_prefixes
#         output_filter = launch_testing_ros.tools.basic_output_filter(
#             filtered_prefixes=get_default_filtered_prefixes() + [
#                 'service not available, waiting again...'
#             ],
#             filtered_rmw_implementation='@rmw_implementation@'
#         )
#         output_files = '@DEMO_NODES_CPP_EXPECTED_OUTPUT@'.split(';')
#         for process, output_file in zip(processes_under_test, output_files):
#             proc_output.assertWaitFor(
#                 expected_output=launch_testing.tools.expected_output_from_file(
#                     path=output_file
#                 ), process=process, output_filter=output_filter, timeout=30
#             )
#         # TODO(hidmic): either make the underlying executables resilient to
#         # interruptions close/during shutdown OR adapt the testing suite to
#         # better cope with it.
#         import time
#         time.sleep(5)


# @launch_testing.post_shutdown_test()
# class TestExecutablesTutorialAfterShutdown(unittest.TestCase):

#     def test_last_process_exit_code(self, proc_info, processes_under_test):
#         """Test last process exit code."""
#         launch_testing.asserts.assertExitCodes(
#             proc_info,
#             process=processes_under_test[-1]
#         )
