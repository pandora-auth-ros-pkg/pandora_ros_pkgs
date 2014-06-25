import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QWidget
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

from pandora_data_fusion_msgs.msg import GlobalProbabilitiesMsg
from std_msgs.msg import Int16
global_propabilities_topic = "/data_fusion/victim_fusion/global_probabilities"
from .widget_info import WidgetInfo


class ProbabilityInfoWidget(QWidget):
    """
   ProbabilityInfoWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin=None):

        super(ProbabilityInfoWidget, self).__init__()
        self._id = "ProbabilityInfo"

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('pandora_rqt_gui'), 'resources', 'ProbabilityInfo.ui')
        loadUi(ui_file, self)
        self.widget_probabilities_info = WidgetInfo(global_propabilities_topic, GlobalProbabilitiesMsg)

        self._timer_refresh_widget = QTimer(self)
        self._timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_probabilities_info.start_monitoring()
        self._timer_refresh_widget.start(500)

    @Slot()
    def refresh_topics(self):

        if self.widget_probabilities_info.last_message is not None:
            self.co2Bar.setValue(self.widget_probabilities_info.last_message.co2 % 100)
            self.thermalBar.setValue(self.widget_probabilities_info.last_message.thermal % 100)
            self.motionBar.setValue(self.widget_probabilities_info.last_message.motion % 100)
            self.soundBar.setValue(self.widget_probabilities_info.last_message.sound % 100)
            self.faceBar.setValue(self.widget_probabilities_info.last_message.face % 100)

    def shutdown(self):
        self.widget_probabilities_info.stop_monitoring()
        self._timer_refresh_widget.stop()
