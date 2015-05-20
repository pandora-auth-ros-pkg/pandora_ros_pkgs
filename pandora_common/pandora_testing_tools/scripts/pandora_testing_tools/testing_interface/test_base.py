# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2014, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Tsirigotis Christos, Peppas Kostas and Lykartsis Giannis"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

from collections import defaultdict
import threading
import time

import unittest

import rospy
import actionlib

from pandora_testing_tools.msg import ReplayBagsAction
from pandora_testing_tools.msg import ReplayBagsGoal
from state_manager.state_client import StateClient

class TestBase(unittest.TestCase):

    def setUp(self):

        for topic in self.messageList.keys():
            self.messageList[topic] = []

    @classmethod
    def mockCallback(cls, data, output_topic):

        rospy.logdebug("Got message from topic : " + str(output_topic))
        rospy.logdebug(data)
        cls.messageList[output_topic].append(data)
        cls.repliedList[output_topic] = True
        cls.block.set()

    def mockPublish(self, input_topic, output_topic, data):

        self.block.clear()
        self.repliedList[output_topic] = False
        self.messageList[output_topic] = list()
        if not isinstance(data, self.publishedTypes[input_topic]):
            rospy.logerr("[mockPublish] Publishes wrong message type.")
        self.publishers[input_topic].publish(data)
        if not self.benchmarking:
            rospy.sleep(self.publish_wait_duration)
        self.block.wait()

    def playFromBag(self, block):

        self.bag_client.send_goal(self.goal)
        if block is True:
            self.bag_client.wait_for_result()

    def assertSimpleResult(self, input_topic, input_data,
                           output_topic, assert_data):

        self.mockPublish(input_topic, output_topic, input_data)
        self.assertTrue(self.repliedList[output_topic])
        self.assertEqual(len(self.messageList[output_topic]), 1)
        output_data = self.messageList[output_topic][0]
        self.assertEqual(output_data, assert_data)

    def simpleBenchmark(self, input_topic, output_topic, data, N=10):

        self.benchmarking = True
        duration = time.time()
        for _ in xrange(N):
            self.mockPublish(input_topic, output_topic, data)
        duration = time.time() - duration
        duration /= N
        return duration * 1000

    @classmethod
    def connect(cls, subscriber_topics, publisher_topics, state, with_bag):

        cls.state_changer = StateClient()
        cls.state_changer.client_initialize()
        rospy.sleep(0.1)
        cls.state_changer.transition_to_state(state)

        cls.repliedList = defaultdict(bool)
        cls.messageList = defaultdict(list)
        cls.subscribers = dict()
        cls.publishedTypes = dict()
        cls.publishers = dict()

        for topic, messagePackage, messageType in subscriber_topics:
            _temp = __import__(messagePackage+'.msg', globals(), locals(), messageType, -1)
            messageTypeObj = getattr(_temp, messageType)
            mock_subscriber = rospy.Subscriber(
                topic, messageTypeObj, cls.mockCallback, topic)
            cls.subscribers[topic] = mock_subscriber

        cls.publish_wait_duration = rospy.Duration(0.2)
        cls.block = threading.Event()
        cls.benchmarking = False
        for topic, messagePackage, messageType in publisher_topics:
            _temp = __import__(messagePackage+'.msg', globals(), locals(), messageType, -1)
            messageTypeObj = getattr(_temp, messageType)
            cls.publishedTypes[topic] = messageTypeObj
            mock_publisher = rospy.Publisher(topic, messageTypeObj)
            cls.publishers[topic] = mock_publisher

        if with_bag:
            cls.bag_client = actionlib.SimpleActionClient('/test/bag_player', ReplayBagsAction)
            cls.bag_client.wait_for_server()
            cls.goal = ReplayBagsGoal()
            cls.goal.start = True
        rospy.sleep(1)

    @classmethod
    def disconnect(cls):

        for mock_subscriber in cls.subscribers.values():
            mock_subscriber.unregister()
        for mock_publisher in cls.publishers.values():
            mock_publisher.unregister()
