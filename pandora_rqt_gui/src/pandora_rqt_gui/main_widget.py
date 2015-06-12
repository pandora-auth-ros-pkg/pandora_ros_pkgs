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

from python_qt_binding.QtCore import QTimer, Slot
from python_qt_binding.QtGui import QWidget
from PyQt4 import QtGui

from .standar_widget import StandarWidget
from .temp_widget import TempWidget
from .co2_widget import CO2Widget
from .battery_widget import BatteryWidget
from .sonars_widget import SonarsWidget


class MainWidget(QWidget):

    def __init__(self, plugin=None):

        super(MainWidget, self).__init__()

        # Widgetlists created for dynamic show
        self.widgetList = []
        self.widgetListId = []

        self.standarWidget = StandarWidget(self)
        self.standarWidget.start()

        # Create and set the Layouts.
        self.vbox = QtGui.QVBoxLayout()
        self.hbox = QtGui.QHBoxLayout()
        self.hbox.addWidget(self.standarWidget)
        self.hbox.addLayout(self.vbox)
        self.setLayout(self.hbox)

        # Timers to refresh 1 sec.
        self.timer_refresh_main_widget = QTimer(self)
        self.timer_refresh_main_widget.timeout.connect(self.main_widget_refresh)
        self.timer_refresh_main_widget.start(1000)

    @Slot()
    def main_widget_refresh(self):
        addwidgetList = []
        removewidgetList = []
        show_all_checked = self.standarWidget.show_all_checked

        # Add the extra widget if checked or remove if unckecked
        if self.standarWidget.temp_checked or show_all_checked:
            addwidgetList.append(TempWidget(self))
        else:
            removewidgetList.append("Temp")

        if self.standarWidget.co2_checked or show_all_checked:
            addwidgetList.append(CO2Widget(self))
        else:
            removewidgetList.append("CO2")

        if self.standarWidget.battery_checked or show_all_checked:
            addwidgetList.append(BatteryWidget(self))
        else:
            removewidgetList.append("Battery")

        if self.standarWidget.sonars_checked or show_all_checked:
            addwidgetList.append(SonarsWidget(self))
        else:
            removewidgetList.append("Sonars")

        # Add if not already added.
        for widget in addwidgetList:
            if widget.id_ not in self.widgetListId:
                self.vbox.addWidget(widget)
                self.widgetList.append(widget)
                self.widgetListId.append(widget.id_)
                widget.start()

        # Remove if not already removed.
        for widget in self.widgetList:
            if widget.id_ in removewidgetList:
                self.vbox.removeWidget(widget)
                self.widgetList.remove(widget)
                self.widgetListId.remove(widget.id_)
                widget.shutdown()
                widget.close()

        self.setLayout(self.hbox)

    def shutdown_plugin(self):

        for widget in self.widgetList:
            widget.shutdown()

        self.standarWidget.shutdown()
