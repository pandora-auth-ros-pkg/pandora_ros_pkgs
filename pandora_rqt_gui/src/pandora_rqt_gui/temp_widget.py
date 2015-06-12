# Software License Agreement
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
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Slot, QTimer
from python_qt_binding.QtGui import QWidget

from .widget_info import WidgetInfo
from pandora_sensor_msgs.msg import ThermalMeanMsg


class TempWidget(QWidget):
    """
    TempWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin=None):

        super(TempWidget, self).__init__()

        # Load Ui and name the widget.
        self.id_ = "Temp"
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('pandora_rqt_gui'), 'resources',
                               'TempWidget.ui')
        loadUi(ui_file, self)

        # Create the subcribers.
        self.widget_info = WidgetInfo("/sensors/thermal_mean", ThermalMeanMsg)

        # Create and connect the timer.
        self.timer_refresh_widget = QTimer(self)
        self.timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info.start_monitoring()
        self.timer_refresh_widget.start(100)

    # Connected slot to the timer in order to refresh.
    @Slot()
    def refresh_topics(self):
        message = self.widget_info.last_message

        if message is not None:
            if message.header.frame_id == 'left_thermal_optical_frame':
                self.lcd1.display(message.thermal_mean)
            elif message.header.frame_id == 'middle_thermal_optical_frame':
                self.lcd2.display(message.thermal_mean)
            elif message.header.frame_id == 'right_thermal_optical_frame':
                self.lcd3.display(message.thermal_mean)

    def shutdown(self):
        """
        Method called when the Widget is terminated.
        """
        self.widget_info.stop_monitoring()
        self.timer_refresh_widget.stop()
