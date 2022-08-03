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
###############################################################################

import traceback

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxyActionClient
from geometry_msgs.msg import TwistStamped

from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient

from flex_bt_msgs.action import BtLoad

from ament_index_python.packages import get_package_share_directory

class BtLoaderState(EventState):

    '''
    Loads Behavior Trees from a files defined by the userdata
    -- bt_topic          string      topic name of the behavior tree server to load a behavior tree
    -- filepaths         string[]    filepaths of the behavior tree defined using Navigation2 XML format
    -- timeout           double      seconds to wait before declaring failure (default: 5.0)
    <= done                           Finished loading behavior trees
    <= failed                         Unable to load behavior trees
    '''

    def __init__(self, bt_topic, filepaths, timeout=5.0 ):
        super(BtLoaderState, self).__init__(outcomes = ['done', 'failed'])

        self._topic = bt_topic
        self._filepaths = filepaths
        self._timeout = timeout
        self._return  = None
        ProxyActionClient._initialize(BtLoaderState._node)

        # Need custom BTLoad action that takes a filename string to load
        self._client = ProxyActionClient({self._topic: BtLoad})


    def execute(self, userdata):

        if self._return:
            # Handle blocked transition by returning previous value
            return self._return

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            ProxyActionClient._result[self._topic] = None # Reset to avoid spam if blocked by low autonomy
            if result.code == 0:
                Logger.loginfo('%s   BT Loading Success!' % (self.name))
                self._return = 'done'
            elif result.code == 1:
                Logger.logerr('%s   Failure' % (self.name))
                self._return =  'failed'
            elif result.code == 2:
                Logger.logerr('%s   Canceled' % (self.name))
                self._return =  'failed'
            else:
                Logger.logerr('%s   Unknown error' % (self.name))
                self._return =  'failed'

        elapsed = self._node.get_clock().now() - self._start_time
        if self._return is None and elapsed.nanoseconds * 1e-9 > self._timeout:
            Logger.logwarn('Timeout waiting to receive result from behavior tree action server' )
            self._return = 'failed'
            return 'failed'

        # Waiting on action results
        return self._return


    def on_enter(self, userdata):
        #upon entering the state will attempt to load the BT
        self._return  = None

        try:
            # Find each file in the workspace shared directory
            for i in range(len(self._filepaths)):
                fileparts = self._filepaths[i].split("/")
                fileparts[0] = get_package_share_directory(fileparts[0])
                self._filepaths[i] = "/".join(fileparts)

            self._client.send_goal(self._topic, BtLoad.Goal(filepaths=self._filepaths))
            Logger.loginfo('loading behavior tree(s) using topic %s ' % (self._topic))
        except Exception as e:
            Logger.logwarn('Was not able to load behavior tree(s) using topic  %s ' % (self._topic))
            Logger.logwarn("Error : %s" % (e))
            self._return = 'failed'

        self._start_time = self._node.get_clock().now()
