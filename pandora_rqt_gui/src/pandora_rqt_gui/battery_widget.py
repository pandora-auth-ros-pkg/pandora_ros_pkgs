# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
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
__author__ = "Chamzas Konstantinos"
__maintainer__ = "Chamzas Konstantinos"
__email__ = "chamzask@gmail.com"

import os
import roslib
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Slot, QTimer
from python_qt_binding.QtGui import QWidget
from rospy.exceptions import ROSException
from pandora_sensor_msgs.msg import BatteryMsg
from .widget_info import WidgetInfo

battery_topic = "sensors/battery"


class BatteryWidget(QWidget):

    """
    BatteryWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin=None):

        super(BatteryWidget, self).__init__()

        # Load Ui and name the widget
        self.id_ = "Battery"
        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('pandora_rqt_gui'),
            'resources', 'BatteryWidget.ui')
        loadUi(ui_file, self)

        # create the subcribers
        self.widget_info_batteries = WidgetInfo(battery_topic, BatteryMsg)

        # create and connect the timer
        self.timer_refresh_widget = QTimer(self)
        self.timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info_batteries.start_monitoring()
        self.timer_refresh_widget.start(100)

    # Connected slot to the timer in order to refresh
    @Slot()
    def refresh_topics(self):

        if self.widget_info_batteries.last_message is not None:

            self.lcd1.display(
                self.widget_info_batteries.last_message.voltage[0])
            self.lcd2.display(
                self.widget_info_batteries.last_message.voltage[1])

            self.PSUBatteryBar.setValue(
                (self.widget_info_batteries.last_message.voltage[0] - 19) * 20)
            self.MotorBatteryBar.setValue(
                (self.widget_info_batteries.last_message.voltage[1] - 19) * 20)

    # Method called when the Widget is terminated
    def shutdown(self):
        self.widget_info_batteries.stop_monitoring()
        self.timer_refresh_widget.stop()
