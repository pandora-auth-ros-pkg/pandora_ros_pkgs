import os
import roslib
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QWidget

from pandora_data_fusion_msgs.msg import GlobalProbabilitiesMsg
from .widget_info import WidgetInfo

global_propabilities_topic = "/data_fusion/signs_of_life"


class ProbabilityInfoWidget(QWidget):
    """
   ProbabilityInfoWidget.start must be called in order to update topic pane.
    """
    def __init__(self, plugin=None):

        super(ProbabilityInfoWidget, self).__init__()
        self.id_ = "ProbabilityInfo"

        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('pandora_rqt_gui'),
            'resources', 'ProbabilityInfo.ui')
        loadUi(ui_file, self)

        #create the subcribers
        self.widget_probabilities_info = WidgetInfo(
            global_propabilities_topic, GlobalProbabilitiesMsg)

        #create and connect the timer
        self.timer_refresh_widget = QTimer(self)
        self.timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_probabilities_info.start_monitoring()
        self.timer_refresh_widget.start(100)

    #Connected slot to the timer in order to refresh
    @Slot()
    def refresh_topics(self):

        if self.widget_probabilities_info.last_message is not None:
            self.co2Bar.setValue(self.widget_probabilities_info.last_message.co2 % 100)
            self.thermalBar.setValue(self.widget_probabilities_info.last_message.thermal % 100)
            self.motionBar.setValue(self.widget_probabilities_info.last_message.motion % 100)
            self.soundBar.setValue(self.widget_probabilities_info.last_message.sound % 100)
            self.faceBar.setValue(self.widget_probabilities_info.last_message.victim % 100)

    def shutdown(self):
        self.widget_probabilities_info.stop_monitoring()
        self.timer_refresh_widget.stop()
