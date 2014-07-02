import os
import roslib
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Slot, QTimer
from python_qt_binding.QtGui import QWidget

from std_msgs.msg import Int16
from .widget_info import WidgetInfo
from pandora_arm_hardware_interface.msg import ThermalMeanMsg


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
        self.widget_info = WidgetInfo("/sensors/thermal_mean", ThermalMeanMsg)

        #create and connect the timer
        self.timer_refresh_widget = QTimer(self)
        self.timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info.start_monitoring()
        self.timer_refresh_widget.start(100)

    #Connected slot to the timer in order to refresh
    @Slot()
    def refresh_topics(self):

        if self.widget_info.last_message is not None:

            if self.widget_info.last_message.header.frame_id == 'left_thermal_optical_frame':
                self.lcd1.display(self.widget_info.last_message.thermal_mean)

            elif self.widget_info.last_message.header.frame_id == 'middle_thermal_optical_frame':
                self.lcd2.display(self.widget_info.last_message.thermal_mean)

            elif self.widget_info.last_message.header.frame_id == 'right_thermal_optical_frame':
                self.lcd3.display(self.widget_info.last_message.thermal_mean)

    #Method called when the Widget is terminated
    def shutdown(self):
        self.widget_info.stop_monitoring()
        self.timer_refresh_widget.stop()
