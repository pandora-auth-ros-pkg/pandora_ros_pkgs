import os
import roslib
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Slot, QTimer
from python_qt_binding.QtGui import QWidget

from rospy.exceptions import ROSException
from std_msgs.msg import Int16
from .widget_info import WidgetInfo
from sensor_msgs.msg import Range

sonars_topic = "sensors/range"


class SonarsWidget(QWidget):
    """
    SonarsWidget.start must be called in order to update topic pane.
    """
    def __init__(self, plugin=None):

        super(SonarsWidget, self).__init__()

        #Load Ui and name the widget
        self.id_ = "Sonars"
        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('pandora_rqt_gui'),
            'resources', 'SonarsWidget.ui')
        loadUi(ui_file, self)

        #create the subcribers
        self.widget_info_sonars = WidgetInfo(sonars_topic, Range)

        #create and connect the timer
        self.timer_refresh_widget = QTimer(self)
        self.timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info_sonars.start_monitoring()
        self.timer_refresh_widget.start(100)

    #Connected slot to the timer in order to refresh
    @Slot()
    def refresh_topics(self):

        if self.widget_info_sonars.last_message is not None:

            if self.widget_info_sonars.last_message.header.frame_id == "left_sonar_frame":
                self.lcd1.display(self.widget_info_sonars.last_message.range)

            elif self.widget_info_sonars.last_message.header.frame_id == "right_sonar_frame":
                self.lcd2.display(self.widget_info_sonars.last_message.range)

    #Method called when the Widget is terminated
    def shutdown(self):
        self.widget_info_sonars.stop_monitoring()
        self.timer_refresh_widget.stop()
