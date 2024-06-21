#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2023
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

from .utility.bt_data_handler import BtDataHandler

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from flex_bt_msgs.action import BtExecute

from ament_index_python.packages import get_package_share_directory


class BtExecuteGoalState(EventState):
    """
    Execute a Behavior Tree from a file defined by the userdata.

    Also allows for setting and getting userdata
    -- bt_topic          string       Topic name of the behavior tree action server
    -- bt_file           string       The behavior tree xml file to execute
    -- goal_id           string       ID of the input goal for the behavior tree to execute with
    -- goal_msg_type     string       The message type for the blackboard to set
    -- request_id        string       ID of the data in the BT Server
    -- request_msg_pkg   string       The package of the requested data ex. PoseStamped pkg is in geometry_msgs
    -- request_msg_type  string       The message type of the requested data ex. PoseStamped
    ># goal              Variety      Either a single goal or list of goals
    #> data              Variety      Requested data from BT Server
    <= done                           Finished behavior tree action
    <= canceled                       Cancel current behavior tree action
    <= failed                         Unable to perform behavior tree action
    """

    def __init__(self, bt_topic, bt_file, goal_id, goal_msg_type, request_id="",
                 request_msg_pkg="", request_msg_type=""):
        super(BtExecuteGoalState, self).__init__(outcomes=['done', 'canceled', 'failed'],
                                                 input_keys=['goal'],
                                                 output_keys=['data'])

        self._topic = bt_topic

        self._goal_id = goal_id
        self._request_id = request_id

        self._request_msg_handler = BtDataHandler(request_msg_type, request_msg_pkg)
        self._goal_msg_handler = BtDataHandler(goal_msg_type)

        self._return = None

        fileparts = bt_file.split("/")
        fileparts[0] = get_package_share_directory(fileparts[0])
        self._file = "/".join(fileparts)

        ProxyActionClient.initialize(BtExecuteGoalState._node)
        self._client = ProxyActionClient({self._topic: BtExecute}, wait_duration=0)

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
                    Logger.logwarn("%s: Unable to create user data for %s: %s - Data request id: %s\n%s" %
                                   (self.name, self._request_msg_handler._msg_type, str(exc), str(self._request_id), str(result)))

            if result.code == 0:
                Logger.loginfo('%s  Success!' % (self.name))
                self._return = 'done'
            elif result.code == 1:
                Logger.logerr('%s   Failure' % (self.name))
                self._return = 'failed'
            elif result.code == 2:
                Logger.logerr('%s   Canceled' % (self.name))
                self._return = 'canceled'
            else:
                Logger.logerr('%s   Unknown error' % (self.name))
                self._return = 'failed'

        return self._return

    def on_enter(self, userdata):
        self._return = None

        try:
            self._goal = BtExecute.Goal(behavior_tree=self._file,
                                        goal_id=self._goal_id,
                                        msg_type=self._goal_msg_handler._msg_type,
                                        result_msg_type=self._request_msg_handler._msg_type,
                                        result_id=self._request_id)

            try:
                goal = userdata.goal
                if type(userdata.goal) is not list:
                    goal = [userdata.goal]

                self._goal.msg_data = self._goal_msg_handler.create_data_string(goal)

            except Exception as exc:
                Logger.logwarn('%s: Unable to set behavior tree goal message data for %s of %s\n%s\n%s '
                               % (self.name, self._topic, self._goal_msg_handler._msg_type, exc, userdata))

            Logger.loginfo('%s: Sending behavior tree goal using topic %s with %s'
                           % (self.name, self._topic, self._goal_msg_handler._msg_type))
            self._client.send_goal(self._topic, self._goal)

        except Exception as exc:
            Logger.logwarn('%s: Was not able to send behavior tree goal using topic  %s ' % (self.name, self._topic))
            Logger.logwarn("Error : %s" % (exc))

    def on_exit(self, userdata):
        if self._topic in ProxyActionClient._result:
            ProxyActionClient._result[self._topic] = None

        if self._client.is_active(self._topic):
            Logger.logerr('%s   Canceling active goal' % (self.name))
            self._client.cancel(self._topic)
