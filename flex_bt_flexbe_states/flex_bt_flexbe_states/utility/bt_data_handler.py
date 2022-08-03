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

import importlib
import rosidl_runtime_py.set_message
import rosidl_runtime_py.convert
import rclpy.time as time
from ament_index_python.packages import get_package_share_directory


class BtDataHandler:

    '''
    Common primitive and ROS message data handling for interfacing with behavior trees
    via the flex_bt_server Action servers.

    -- msg_type  string   The message type of the requested data (ex. PoseStamped or double[])
    -- msg_pkg   string   The package of the message ex. PoseStamped pkg is geometry_msgs
    '''

    _primitives = ("string", "char", "bool", "int", "double", "float")#, "int[]", "double[]", "float[]")
    _primitive_classes = {"string":str, "char":str, "bool":bool, "int":int, "double":float, "float":float}

    def __init__(self, msg_type="", msg_pkg=""):

        self._msg_pkg  = msg_pkg
        self._msg_type = msg_type.lower()
        self._msg_base_type = msg_type.lower()
        if "[]" in self._msg_base_type:
            self._msg_base_type = self._msg_base_type.replace("[]", "")

        self._is_floating_point  = self._msg_base_type == "float" or self._msg_base_type == "double"

        self._msg_base_class = None
        if self._msg_base_type == "path":
            try:
                if msg_pkg == "":
                    msg_pkg = "geometry_msgs"
                    self._msg_pkg  = msg_pkg

                pose_module = importlib.import_module("geometry_msgs.msg")
                self._path_pose_class = getattr(pose_module, "PoseStamped")
            except Exception as exc:
                raise ValueError(' Failed to load pose class for type %s/%s.msg\n %s' % (msg_pkg, msg_type, str(exc)))
        else:
            self._path_pose_class = None


        if self._msg_base_type not in self._primitives:
            if msg_pkg != "" and msg_type != "":
                try:
                    msg_module = importlib.import_module('%s.msg' % msg_pkg)
                    self._msg_base_class = getattr(msg_module, msg_type)
                except Exception as exc:
                    raise ValueError(' Failed to load message class for type %s/%s.msg\n %s' % (msg_pkg, msg_type, str(exc)))
                self.create_data_from_result = self._create_msgs_from_data_string_list
            else:
                # No message type is defined, but that is OK
                pass
        else:
            # Define handlers for the primitives
            self._msg_base_class = BtDataHandler._primitive_classes[self._msg_base_type]
            self.create_data_from_result = self._create_primitve_from_data_string_list


    @staticmethod
    def convert_primitives(data):
        '''
        Convert data to standard string format for data handling
        '''
        if type(data) is list:
            return ";".join(str(element) for element in data)
        else:
            return str(data)


    @staticmethod
    def split_data_string(data_string):
        try:
            data = data_string.strip().split(";")
            # Note: empty string from ;; used to indicate end of sub message
            return data

        except Exception as exc:
            raise ValueError(' Debug split_data_string \n   Data (%s)\n%s' % (str(data_string), exc))

    def _create_primitve_from_data_string_list(self, data_string_list):
        '''
        Given a list of formated data_string from BtServer,
        convert to specified data type.
            NOTE: Presumed called as create_primitve_from_data_string(result.result_data)

        @param data_string  ['formated_string']
        @return appropriate data
        '''

        assert len(data_string_list) == 1, f"Calling create_primitve with list of length={len(data_string_list)}"
        data = BtDataHandler.split_data_string(data_string_list[0])

        if data[-1] == "":
            # Last element as empty string denotes end of data structure for messages
            # Not used in primitive processing
            data = data[:-1]

        if self._is_floating_point:
            floats = []
            for item in data:
                floats.append(float(item))

            if len(floats) == 1:
                return floats[0]
            return floats

        elif self._msg_type == "char":
            return chr(data[0])

        elif self._msg_type == "bool":
            # Accept True/T or False/F
            if data[0].capitalize()[0] == "T":
                return True
            elif data[0].capitalize()[0] == "F":
                return False
            else:
                raise ValueError(f"Invalid data for bool type ({data[0]})")

        elif self._msg_base_type == "int" :
            ints = []
            for item in data:
                ints.append(int(item))

            if len(ints) == 1:
                return ints[0]

            return ints
        else:
            # Treat as string type
            return data


    def _create_msgs_from_data_string_list(self, data_string_list):
        '''
        Given a list of formated data_string from BtServer,
        convert to specified data type.
            NOTE: Presumed called as create_msgs_from_data_string_list(result.result_data)

        @param data_string  ['formated_string']
        @return appropriate data
        '''

        try:

            messages = []

            if self._msg_type == "path":
                msg = self._msg_base_class()

                msg_data = BtDataHandler.split_data_string(data_string_list[0])
                header, msg_data = self.populate_msg(msg.header, msg_data)
                msg.header = header

                pose_class = self._path_pose_class

                # Process poses from remaining data_strings
                for ndx in range(1, len(data_string_list)):
                    msg_data = BtDataHandler.split_data_string(data_string_list[ndx])
                    pose, _ = self.populate_msg(pose_class(), msg_data)
                    msg.poses.append(pose)

                messages.append(msg)
            else:
                for data_string in data_string_list:
                    msg_data = BtDataHandler.split_data_string(data_string)
                    print(f" populate {self._msg_type} msg with {msg_data}")
                    msg, _ = self.populate_msg(self._msg_base_class(), msg_data)
                    messages.append(msg)

            if len(messages) == 1:
                return messages[0]

            return messages
        except Exception as exc:
            raise ValueError(f"  Failed to create message from data string <{data_string_list}>\n{exc}")

    @staticmethod
    def populate_msg(message, data):
        fields = rosidl_runtime_py.convert.get_message_slot_types(message)
        base_msg = True
        fields_dict = {}

        for key in fields.keys():
            try:
                if str(key) != "stamp":
                    subfields = rosidl_runtime_py.convert.get_message_slot_types(getattr(message, key))
                    base_msg = False

                    sub_msg, new_data = BtDataHandler.populate_msg(getattr(message, key), data)

                    setattr(message, key, sub_msg)
                    data = new_data
            except:
                continue

        if base_msg:
            for i in range(data.index("")):
                field_data = data[i]
                field_data = field_data.split(":")

                if field_data[0] == "stamp":
                    try:
                        fields_dict[field_data[0]] = time.Time(nanoseconds=int(field_data[1])).to_msg()
                    except:
                        # No node access to time stamp as current node time, so just flag
                        fields_dict[field_data[0]] = time.Time(nanoseconds=0).to_msg()
                else:
                    fields_dict[field_data[0]] = field_data[1]

            if "stamp" in fields_dict:
                for key in fields_dict.keys():
                    setattr(message, key, fields_dict[key])
            else:
                rosidl_runtime_py.set_message.set_message_fields(message, fields_dict)

            new_data_list = data[data.index("")+1:]
            return message, new_data_list

        return message, data

    def create_data_string(self, data_list):
        """
        Create data string for BtServer from list of messages
        """
        msg_data = []

        if self._msg_type in self._primitives:
            msg_data.append(self.convert_primitives(data_list))
        elif self._msg_type == "path":
            path = data_list[0]
            assert self._msg_type in path.__class__.__name__.lower() , f"Goal type mis-match {path.__class__.__name__} not Path!"
            fields = rosidl_runtime_py.convert.get_message_slot_types(path)
            msg_data.append(BtDataHandler.get_fields(getattr(path, "header"), ""))
            for pose in getattr(path, "poses"):
                msg_data.append(BtDataHandler.get_fields(pose, ""))
        else:
            for item in data_list:
                fields_data = BtDataHandler.get_fields(item, "")
                msg_data.append(fields_data)

        return msg_data

    @staticmethod
    def get_fields(message, data):
        '''
        Extract data fields from message and add to string in standard format
        '''
        try:

            # fields is an OrderedDict that preserves key order
            fields = rosidl_runtime_py.convert.get_message_slot_types(message)

            for key in fields.keys():
                if str(key) == "stamp":
                    # Represent as single uint64_t nanoseconds
                    data += f"{getattr(message, key).sec*(10**9) + getattr(message, key).nanosec};"
                else:
                    data = BtDataHandler.get_fields(getattr(message, key), data)
        except AttributeError:
            # Normal at bottom primitive in recursive call
            data += str(message) + ";"
        except Exception as exc:
            print(f" Exception in get_fields {type(exc)}: {exc} - just use raw message")
            data += str(message) + ";"

        return data

    @staticmethod
    def check_fields(message1, message2):
        '''
        Validate data fields in two messages of same type
        '''
        if not isinstance(message1, message2.__class__):
            print("Messages must be same type to compare")
            return False

        try:

            # fields is an OrderedDict that preserves key order
            fields = rosidl_runtime_py.convert.get_message_slot_types(message1)

            equals = True
            for key in fields.keys():
                if str(key) == "stamp":
                    stamp1 = getattr(message1, key)
                    stamp2 = getattr(message2, key)
                    equals = equals and stamp1.sec == stamp2.sec
                    equals = equals and stamp1.nanosec == stamp2.nanosec
                    if not equals:
                        print(f"Not equal at {key}: {message1.stamp} {message2.stamp}")
                        return False
                else:
                    equals = equals and BtDataHandler.check_fields(getattr(message1, key), getattr(message2, key))

                if not equals:
                    print(f"Not equal at {key}")
                    return False

            return equals
        except AttributeError:
            # Normal at bottom primitive in recursive call
            return message1 == message2
        except Exception as exc:
            print(f"Unknown exception {exc} in check_fields: {message1} {message2}")
            return False
