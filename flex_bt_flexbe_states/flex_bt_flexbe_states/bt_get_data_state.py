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

from .utility.bt_data_handler import BtDataHandler

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from flex_bt_msgs.action import BtGetData

from ament_index_python.packages import get_package_share_directory


class BtGetDataState(EventState):

    '''
    Gets userdata from the BT Server's blackboard
    -- bt_topic          string       Topic name of the behavior tree action server
    -- request_id        string       ID of the data in the BT Server
    -- request_msg_pkg   string       The package of the requested data ex. PoseStamped pkg is in geometry_msgs
    -- request_msg_type  string       The message type of the requested data ex. PoseStamped
    #> data              Variety      Requested data from BT Server
    <= done                           Finished behavior tree action
    <= failed                         Unable to perform behavior tree action
    '''

    def __init__(self, bt_topic, request_id, request_msg_pkg, request_msg_type):
        super(BtGetDataState, self).__init__(outcomes = ['done', 'failed'], output_keys=['data'])

        self._topic = bt_topic
        self._request_id = request_id

        self._request_msg_handler = BtDataHandler(request_msg_type, request_msg_pkg)

        ProxyActionClient._initialize(BtGetDataState._node)
        self._client = ProxyActionClient({self._topic: BtGetData})
        self._return  = None


    def execute(self, userdata):

        if self._return:
            # Handle blocked transition by returning previous value
            return self._return

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            ProxyActionClient._result[self._topic] = None

            if self._request_msg_handler._msg_base_class:
                try:
                    userdata.data = self._request_msg_handler.create_data_from_result(result.result_data)
                except Exception as exc:
                    Logger.logwarn("%s: Unable to create user data for %s: %s\n%s" %
                            (self.name, self._request_msg_handler._msg_type, str(exc), str(result)))

            if result.code == 0:
                Logger.loginfo('%s  Success!' % (self.name))
                self._return = 'done'
            elif result.code == 1:
                Logger.logerr('%s   Failure' % (self.name))
                self._return =  'failed'
            else:
                Logger.logerr('%s   Unknown error' % (self.name))
                self._return =  'failed'

        return self._return


    def on_enter(self, userdata):
        self._return  = None

        try:
            self._goal = BtGetData.Goal(request_id=self._request_id, request_msg_type=self._request_msg_handler._msg_type)
            Logger.loginfo('%s: Sending behavior tree goal using topic %s ' % (self.name, self._topic))
            self._client.send_goal(self._topic, self._goal)

        except Exception as e:
            Logger.logwarn('%s: Was not able to send behavior tree goal using topic  %s ' % (self.name, self._topic))
            Logger.logwarn("Error : %s" % (e))


    def on_exit(self, userdata):
        if self._topic in ProxyActionClient._result:
            ProxyActionClient._result[self._topic] = None

        if self._client.is_active(self._topic):
            Logger.logerr('%s   Canceling active goal' % (self.name))
            self._client.cancel(self._topic)
