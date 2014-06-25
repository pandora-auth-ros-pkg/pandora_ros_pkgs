#!/usr/bin/env python
from StringIO import StringIO
from std_msgs.msg import String
import roslib
import rospy


class WidgetInfo():

    def __init__(self, topic_name, topic_type):

        self._topic_name = topic_name
        self.error = None
        self._subscriber = None
        self.monitoring = False
        self.message_class = topic_type
        self.last_message = None

    def toggle_monitoring(self):
        if self.monitoring:
            self.stop_monitoring()
        else:
            self.start_monitoring()

    def start_monitoring(self):
        if self.message_class is not None:
            self.monitoring = True
            self._subscriber = rospy.Subscriber(self._topic_name, self.message_class, self.message_callback)

    def stop_monitoring(self):
        self.monitoring = False
        if self._subscriber is not None:
            self._subscriber.unregister()

    def message_callback(self, message):

        #~ rospy.loginfo(rospy.get_caller_id()+" I heard %d ",message.data)
        self.last_message = message
