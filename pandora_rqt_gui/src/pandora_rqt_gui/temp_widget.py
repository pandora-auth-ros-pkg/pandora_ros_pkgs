import os
import roslib
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Slot, QTimer
from python_qt_binding.QtGui import QWidget

from std_msgs.msg import Int16
from .widget_info import WidgetInfo


class TempWidget(QWidget):
    """
    TempWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin=None):

        super(TempWidget, self).__init__()

        #Load Ui and name the widget
        self.id_ = "Temp"
        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('pandora_rqt_gui'),
            'resources', 'TempWidget.ui')
        loadUi(ui_file, self)

        #create the subcribers
        self.widget_info_left = WidgetInfo("chatter", Int16)
        self.widget_info_center = WidgetInfo("chatter", Int16)
        self.widget_info_right = WidgetInfo("chatter", Int16)

        #create and connect the timer
        self.timer_refresh_widget = QTimer(self)
        self.timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info_left.start_monitoring()
        self.widget_info_center.start_monitoring()
        self.widget_info_right.start_monitoring()
        self.timer_refresh_widget.start(1000)

    #Connected slot to the timer in order to refresh
    @Slot()
    def refresh_topics(self):

        if self.widget_info_left.last_message is not None:
            self.lcd1.display(self.widget_info_left.last_message.data)

        if self.widget_info_center.last_message is not None:
            self.lcd2.display(self.widget_info_center.last_message.data)

        if self.widget_info_right.last_message is not None:
            self.lcd3.display(self.widget_info_right.last_message.data)

    #Method called when the Widget is terminated
    def shutdown(self):
        self.widget_info_left.stop_monitoring()
        self.widget_info_center.stop_monitoring()
        self.widget_info_right.stop_monitoring()
        self.timer_refresh_widget.stop()
