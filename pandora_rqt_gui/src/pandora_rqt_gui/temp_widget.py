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


class TempWidget(QWidget):
    """
    TempWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin = None ):
       
       
        super(TempWidget, self).__init__()
        
        self._id= "Temp"
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('pandora_rqt_gui'), 'resources', 'TempWidget.ui')
        loadUi(ui_file, self)
        self.widget_info_left = WidgetInfo("chatter", Int16 )
        self.widget_info_center = WidgetInfo("chatter", Int16 )
        self.widget_info_right = WidgetInfo("chatter", Int16 )
        
        
        self._timer_refresh_widget = QTimer(self)
        self._timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info_left.start_monitoring()
        self.widget_info_center.start_monitoring()
        self.widget_info_right.start_monitoring()
        self._timer_refresh_widget.start(1000)

    @Slot()
    def refresh_topics(self):
      
      if self.widget_info_left.last_message is not None:
        self.lcd1.display(self.widget_info_left.last_message.data)
      if self.widget_info_center.last_message is not None:
        self.lcd2.display(self.widget_info_center.last_message.data)
      if self.widget_info_right.last_message is not None:
        self.lcd3.display(self.widget_info_right.last_message.data)
       
       
    def shutdown(self):
        self.widget_info_left.stop_monitoring()
        self.widget_info_center.stop_monitoring()
        self.widget_info_right.stop_monitoring()
        self._timer_refresh_widget.stop()
  
      
