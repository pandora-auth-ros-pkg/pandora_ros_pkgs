#!/usr/bin/env python
from std_msgs.msg import String

import roslib
import rospy


class WidgetInfo():

    def __init__(self, topic_name, topic_type):

        self.topic_name_ = topic_name
        self.error = None
        self.subscriber = None
        self.monitoring = False
        self.topic_type_ = topic_type
        self.last_message = None

    def toggle_monitoring(self):

        if self.monitoring:
            self.stop_monitoring()
        else:
            self.start_monitoring()

    def start_monitoring(self):

        if self.topic_type_ is not None:
            self.monitoring = True
            self.subscriber = rospy.Subscriber(
                self.topic_name_,
                self.topic_type_,
                self.message_callback)

    def stop_monitoring(self):
        self.monitoring = False

        if self.subscriber is not None:
            self.subscriber.unregister()

    def message_callback(self, message):
        self.last_message = message
