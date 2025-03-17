# Copyright 2023 Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""State tests for launch in flex_bt_flexbe_states."""

from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch method."""
    flexbe_testing_dir = get_package_share_directory('flexbe_testing')
    flex_bt_states_test_dir = get_package_share_directory('flex_bt_flexbe_states')

    path = join(flex_bt_states_test_dir, 'test')

    testcases = ''
    testcases += join(path, 'bt_execute_goal_state.test') + '\n'
    testcases += join(path, 'bt_execute_state.test') + '\n'
    testcases += join(path, 'bt_get_data_state.test') + '\n'
    testcases += join(path, 'bt_loader_state.test') + '\n'
    testcases += join(path, 'bt_set_data_state.test') + '\n'

    return LaunchDescription([
        DeclareLaunchArgument('pkg', default_value='flex_bt_flexbe_states'),
        DeclareLaunchArgument('testcases', default_value=testcases),
        DeclareLaunchArgument('compact_format', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(flexbe_testing_dir, 'launch', 'flexbe_testing.launch.py')),
            launch_arguments={
                'package': LaunchConfiguration('pkg'),
                'compact_format': LaunchConfiguration('compact_format'),
                'testcases': LaunchConfiguration('testcases'),
            }.items()
        )
    ])
