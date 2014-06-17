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


#~ from data_fusion_communications.msg import *
from .widget_info import WidgetInfo


class CO2Widget(QWidget):
    """
    CO2Widget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin = None ):
       
       
        super(CO2Widget, self).__init__()
        
        self._id= "CO2"
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('pandora_rqt_gui'), 'resources', 'CO2Widget.ui')
        
        loadUi(ui_file, self)
        self.widget_info= WidgetInfo("chatter", Int16 )
        self.lcd.display(self.widget_info.last_message)
        
        self._timer_refresh_widget = QTimer(self)
        self._timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info.start_monitoring()
        self._timer_refresh_widget.start(1000)

    @Slot()
    def refresh_topics(self):
      
      if self.widget_info.last_message is not None:
          self.lcd.display(self.widget_info.last_message.data)
       
       
    def shutdown(self):
        
        self.widget_info.stop_monitoring()
        self._timer_refresh_widget.stop()
  
      
