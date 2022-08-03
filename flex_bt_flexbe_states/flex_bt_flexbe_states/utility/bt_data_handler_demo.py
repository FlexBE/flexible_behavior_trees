#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2022
#  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
#  Christopher Newport University
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#       POSSIBILITY OF SUCH DAMAGE.
###############################################################################


from bt_data_handler import BtDataHandler

if __name__ == '__main__':

    # Some demonstrations of the BtDataHandler class

    dh_double = BtDataHandler(msg_pkg="", msg_type="double")
    data = dh_double.create_data_from_result(["3.1415927;"])
    print(f"Type {type(data)} : {data}")

    dh_float_list = BtDataHandler(msg_pkg="", msg_type="float[]")
    data = dh_float_list.create_data_from_result(["3.1415927; 1.57; 6.28; -1.0"])
    print(f"Type {type(data)} {type(data[0])} : {data}")

    dh_bool = BtDataHandler(msg_pkg="", msg_type="bool")
    data = dh_bool.create_data_from_result(["true"])
    print(f"Type {type(data)} : {data}")
    data = dh_bool.create_data_from_result(["F"])
    print(f"Type {type(data)} : {data}")
    data = dh_bool.create_data_from_result(["True"])
    print(f"Type {type(data)} : {data}")

    dh_pose = BtDataHandler(msg_pkg="geometry_msgs", msg_type="PoseStamped")
    pose_string = f'stamp:{123*(10**9) + 5*(10**8)};frame_id:map;;' + \
                  f'x:1.0;y:2.0;z:3.0;;x:0.1;y:0.2;z:0.3;w:0.4;;'
    data = dh_pose.create_data_from_result([pose_string])
    print(f"\n\n\nType {type(data)} : {data}")

    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header.stamp.sec = 123
    pose.header.stamp.nanosec = 500000000
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.
    pose.pose.position.y = 2.
    pose.pose.position.z = 3.
    pose.pose.orientation.x = 0.1
    pose.pose.orientation.y = 0.2
    pose.pose.orientation.z = 0.3
    pose.pose.orientation.w = 0.4
    print(f"\n\nNative {type(pose)} : {data}")
    if not dh_pose.check_fields(pose, data):
        print("\n\n\nConversion error!")
        exit(-1)
    else:
        print("\n\nValid data conversion for PoseStamped!")
        print("  ", pose.pose.position)
        print("  ", data.pose.position)

    data_string_list = dh_pose.create_data_string([pose])
    print("\n\nPoseStamped data string from message: ", data_string_list)

    print("\n\nPath:")
    dh_path = BtDataHandler(msg_pkg="nav_msgs", msg_type="Path")
    path_result = ['stamp:;frame_id:map;',
        'stamp:;frame_id:;;x:-0.552;y:-0.177;z:0.00;;x:0.00;y:0.00;z:0.00;w:1.00;;',
        'stamp:;frame_id:;;x:-0.553;y:-0.152;z:0.00;;x:0.00;y:0.00;z:0.00;w:1.00;;',
        'stamp:;frame_id:;;x:-0.554;y:-0.127;z:0.00;;x:0.00;y:0.00;z:0.00;w:1.00;;',
        'stamp:;frame_id:;;x:-0.555;y:-0.103;z:0.00;;x:0.00;y:0.00;z:0.00;w:1.00;;',
        'stamp:;frame_id:;;x:-0.556;y:-0.078;z:0.00;;x:0.00;y:0.00;z:0.00;w:1.00;;',
        'stamp:;frame_id:;;x:-0.602;y:0.689;z:0.00;;x:0.00;y:0.00;z:0.00;w:1.00;;',
        'stamp:;frame_id:;;x:-0.598;y:0.714;z:0.00;;x:0.00;y:0.00;z:0.00;w:1.00;;',
        'stamp:;frame_id:;;x:-0.595;y:0.739;z:0.00;;x:0.00;y:0.00;z:0.00;w:1.00;;',
        'stamp:;frame_id:;;x:-0.645;y:0.789;z:0.00;;x:0.00;y:0.00;z:0.00;w:1.00;;',
        'stamp:;frame_id:;;x:-0.646;y:0.789;z:0.00;;x:0.00;y:0.00;z:0.571;w:0.821;;']
    data = dh_path.create_data_from_result(path_result)
    print(f"Type {type(data)} : {data.header}")
    for pose in data.poses:
        print(pose)
