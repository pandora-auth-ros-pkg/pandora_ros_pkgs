import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QWidget
from PyQt4 import QtGui

import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException
from .standar_widget import StandarWidget
from .temp_widget import TempWidget
from .co2_widget import CO2Widget
from .battery_widget import BatteryWidget
from .sonars_widget import SonarsWidget


class MainWidget(QWidget):

    def __init__(self, plugin=None):

        super(MainWidget, self).__init__()

        # Widgetlists created for dynamic show
        self._widgetList = []
        self._widgetListId = []

        self._standarWidget = StandarWidget(self)
        self._standarWidget.start()

        #Create and set the Layouts
        self._vbox = QtGui.QVBoxLayout()
        self._hbox = QtGui.QHBoxLayout()
        self._hbox.addWidget(self._standarWidget)
        self._hbox.addLayout(self._vbox)
        self.setLayout(self._hbox)

        #Timers  to refresh 1 sec
        self._timer_refresh_main_widget = QTimer(self)
        self._timer_refresh_main_widget.timeout.connect(self.main_widget_refresh)
        self._timer_refresh_main_widget.start(1000)

    @Slot()
    def main_widget_refresh(self):
        addwidgetList = []
        removewidgetList = []

        # Add the extra widget if checked or remove if unckecked
        if self._standarWidget._tempChecked or self._standarWidget._showAllChecked:
            addwidgetList.append(TempWidget(self))
        else:
            removewidgetList.append("Temp")

        if self._standarWidget._co2Checked or self._standarWidget._showAllChecked:
            addwidgetList.append(CO2Widget(self))
        else:
            removewidgetList.append("CO2")

        if self._standarWidget._batteryChecked or self._standarWidget._showAllChecked:
            addwidgetList.append(BatteryWidget(self))
        else:
            removewidgetList.append("Battery")

        if self._standarWidget._sonarsChecked or self._standarWidget._showAllChecked:
            addwidgetList.append(SonarsWidget(self))
        else:
            removewidgetList.append("Sonars")

        # Add if not already added
        for widget in addwidgetList:
            if widget._id not in self._widgetListId:
                self._vbox.addWidget(widget)
                self._widgetList.append(widget)
                self._widgetListId.append(widget._id)
                widget.start()

        #remove if not already removed
        for widget in self._widgetList:
            if widget._id in removewidgetList:
                self._vbox.removeWidget(widget)
                self._widgetList.remove(widget)
                self._widgetListId.remove(widget._id)
                widget.shutdown()
                widget.close()

        self.setLayout(self._hbox)

    def shutdown_plugin(self):

        for widget in self._widgetList:
            widget.shutdown()

        self._standarWidget.shutdown()
