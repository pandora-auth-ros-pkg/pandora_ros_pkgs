import os
import roslib
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Slot, QTimer
from python_qt_binding.QtGui import QWidget

from .widget_info import WidgetInfo
from pandora_arm_hardware_interface.msg import Co2Msg

co2_topic = "sensors/co2"


class CO2Widget(QWidget):
    """
    BatteryWidget.start must be called in order to update topic pane.
    """
    def __init__(self, plugin=None):

        super(CO2Widget, self).__init__()

        #Load Ui and name the widget
        self.id_ = "CO2"
        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('pandora_rqt_gui'),
            'resources', 'CO2Widget.ui')
        loadUi(ui_file, self)

        #create the subcribers
        self.widget_info = WidgetInfo(co2_topic, Co2Msg)

        #create and connect the timer
        self.timer_refresh_widget = QTimer(self)
        self.timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info.start_monitoring()
        self.timer_refresh_widget.start(1000)

    #Connected slot to the timer in order to refresh
    @Slot()
    def refresh_topics(self):
        if self.widget_info.last_message is not None:
            self.lcd.display(self.widget_info.last_message.co2_percentage)

    #Method called when the Widget is terminated
    def shutdown(self):
        self.widget_info.stop_monitoring()
        self.timer_refresh_widget.stop()
