from __future__ import division
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QHeaderView, QIcon, QMenu, QTreeWidgetItem, QWidget
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Int16

from .widget_info import WidgetInfo


class BatteryWidget(QWidget):
    """
    BatteryWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin = None ):
       
       
        super(BatteryWidget, self).__init__()
        
        self._id= "Battery"
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('pandora_rqt_gui'), 'resources', 'BatteryWidget.ui')
        
        loadUi(ui_file, self)
        self.widget_info_PSU = WidgetInfo("chatter", Int16 )
        self.widget_info_Motor = WidgetInfo("chatter",Int16)
        
        
        self._timer_refresh_widget = QTimer(self)
        self._timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info_PSU.start_monitoring()
        self.widget_info_Motor.start_monitoring()
        self._timer_refresh_widget.start(1000)

    @Slot()
    def refresh_topics(self):
      
      if self.widget_info_PSU.last_message is not None:
        self.lcd1.display(self.widget_info_PSU.last_message.data)
        self.PSUBatteryBar.setValue(self.widget_info_PSU.last_message.data)
      if self.widget_info_Motor.last_message is not None:
        self.lcd2.display(self.widget_info_Motor.last_message.data)
        self.MotorBatteryBar.setValue(self.widget_info_Motor.last_message.data)
       
       
    def shutdown(self):
        self.widget_info_PSU.stop_monitoring()
        self.widget_info_Motor.stop_monitoring()
        self._timer_refresh_widget.stop()
       
  
      
