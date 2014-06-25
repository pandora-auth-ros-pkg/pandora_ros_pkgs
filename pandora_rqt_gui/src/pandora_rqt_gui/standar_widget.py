import os


import roslib
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot, QTime
from python_qt_binding.QtGui import QWidget, QPixmap

from rospy.exceptions import ROSException

from std_msgs.msg import Int32
from pandora_data_fusion_msgs.msg import WorldModelMsg

from .widget_info import WidgetInfo
from .victim_found_server import ValidateVictimActionServer
from .probability_info import ProbabilityInfoWidget
from .console import Console
from .gui_state_client import GuiStateClient

world_model_info_topic = '/data_fusion/world_model_info'
robocup_score_topic = 'data_fusion/robocup_score'

class StandarWidget(QWidget):
    """
    StandarWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin=None):

        super(StandarWidget, self).__init__()

        #Load Ui
        self._rp = rospkg.RosPack()
        ui_file = os.path.join(self._rp.get_path('pandora_rqt_gui'), 'resources', 'StandarWidget.ui')
        loadUi(ui_file, self)

        self._Console = Console()
        self._ConsoleWidget = self._Console._widget
        self._ProbabilityInfoWidget = ProbabilityInfoWidget(self)
        self._GuiStateClient =  GuiStateClient()

        #Add Console in the 2,1 position of InternalGrid
        self.internalGrid.addWidget(self._ConsoleWidget, 2, 1)

        #use full ABSOLUTE path to the image, not relative
        self.image.setPixmap(QPixmap(os.path.join(self._rp.get_path('pandora_rqt_gui'), 'images', 'pandora_logo.jpeg')))

        # the ValidateVictimActionServer is used called when a victim is found
        self.ValidateVictimActionServer = ValidateVictimActionServer('victimValidation')

        #Subscribe the score the world_model_info and Info
        self.score_info = WidgetInfo(robocup_score_topic, Int32)
        self.world_model_info = WidgetInfo(world_model_info_topic, WorldModelMsg)
        self._victimInfo = []

        self.timerStarted = False

        #Connecting the Buttons
        self.go_button.clicked.connect(self.go_button_clicked)
        self.stop_button.clicked.connect(self.stop_button_clicked)
        self.confirm_button.clicked.connect(self.confirm_victim_clicked)
        self.decline_button.clicked.connect(self.decline_button_clicked)
        self.left_panel_button.clicked.connect(self.left_panel_button_clicked)
        self.reset_timer_button.clicked.connect(self.reset_timer_button_clicked)
        self.save_geotiff_button.clicked.connect(self.save_geotiff_button_clicked)
        self.autonomous_state_button.clicked.connect(self.autonomous_state_button_clicked)
        self.teleop_state_button.clicked.connect(self.teleop_state_button_clicked)
        self.semi_autonomous_state.clicked.connect(self.semi_autonomous_state_clicked)
        self.search_and_rescue_button.clicked.connect(self.search_and_rescue_button_clicked)
        self.mapping_mission_button.clicked.connect(self.mapping_mission_button_clicked)
        
        #Connecting the CheckBoxes
        self.tempCheckBox.stateChanged.connect(self.temp_checked)
        self.co2CheckBox.stateChanged.connect(self.co2_checked)
        self.sonarsCheckBox.stateChanged.connect(self.sonars_checked)
        self.batteryCheckBox.stateChanged.connect(self.battery_checked)
        self.showAllCheckBox.stateChanged.connect(self.show_all)

        #In the Beggining all the checkBoxes are unchecked
        self._tempChecked = False
        self._batteryChecked = False
        self._sonarsChecked = False
        self._co2Checked = False
        self._showAllChecked = False
        
        #The left panel is visible
        self._left_panel = True
        
        #The state is Autonomous
        self._autonomous = True

        # Refresh timer
        self._timer_refresh_widget = QTimer(self)
        self._timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):

        self.score_info.start_monitoring()
        self.world_model_info.start_monitoring()
        self._timer_refresh_widget.start(1000)

    def enableVictimFoundOptions(self):

        self.confirm_button.setEnabled(True)
        self.decline_button.setEnabled(True)
        self.victimx.setEnabled(True)
        self.victimy.setEnabled(True)
        self.victimz.setEnabled(True)
        self.sensorID.setEnabled(True)
        self.probability.setEnabled(True)
        self.setVictimInfo()
        self._ProbabilityInfoWidget.show()
        self.internalGrid.addWidget(self._ProbabilityInfoWidget, 1, 0)
        self._ProbabilityInfoWidget.start()

    def disableVictimFoundOptions(self):

        self.confirm_button.setEnabled(False)
        self.decline_button.setEnabled(False)
        self.victimx.setEnabled(False)
        self.victimy.setEnabled(False)
        self.victimz.setEnabled(False)
        self.sensorID.setEnabled(False)
        self.probability.setEnabled(False)
        self.internalGrid.removeWidget(self._ProbabilityInfoWidget)
        self._ProbabilityInfoWidget.shutdown()
        self._ProbabilityInfoWidget.close()

    def setVictimInfo(self):

        self.victimx.setText(" Victim X Position " + str(self._victimInfo[0]))
        self.victimy.setText(" Victim Y Position " + str(self._victimInfo[1]))
        self.victimz.setText(" Victim Z Position " + str(self._victimInfo[2]))
        sensors = "";
        for sensor in self._victimInfo[4]:
          sensors = sensors+(sensor)+" " 

        self.sensorID.setText(" Sensor IDs " + "".join(sensors))
        self.probability.setText(" Probability " + str(self._victimInfo[3]))

    # Refreshing the topics
    @Slot()
    def refresh_topics(self):

        self.state.setText(self._GuiStateClient.getState())
        
        if self.score_info.last_message is not None:
            self.score.display(self.score_info.last_message.data)
            
        if self.world_model_info.last_message is not None:
            victims_number = 0
            for victim in self.world_model_info.last_message.visitedVictims:
                if victim.valid:
                    victims_number = victims_number + 1
            self.victimsFound.display(victims_number)

        if self.timerStarted:
            self._time = self._time.addSecs(-1)
            self.timer.setTime(self._time)

        # Enable the victim found Options if it is found
        if self.ValidateVictimActionServer._victimFound:
            self._victimInfo = self.ValidateVictimActionServer._victimInfo
            self.enableVictimFoundOptions()
        

    #Start the timer
    def go_button_clicked(self):

        self.timerStarted = True
        self._time = (self.timer.time())
        self.timer.setTime(self._time)

    #Stop the timer and The robot
    def stop_button_clicked(self):

        self.timerStarted = False
        self._GuiStateClient.transition_to_state(10)

    def decline_button_clicked(self):
        self.ValidateVictimActionServer._victimValid = False
        self.disableVictimFoundOptions()
        self.ValidateVictimActionServer._operatorResponded = True
        self.ValidateVictimActionServer._victimFound = False

    def confirm_victim_clicked(self):
        self.ValidateVictimActionServer._victimValid = True
        self.ValidateVictimActionServer._operatorResponded = True
        self.disableVictimFoundOptions()
        self.ValidateVictimActionServer._victimFound = False

    def left_panel_button_clicked(self):
        if self._left_panel:
            self.image.close()
            self.victimInfo_2.close()
            self.left_panel_button.setText("ShowLeftPanel")
            self._left_panel = False
        else :
            self.image.show()
            self.victimInfo_2.show()
            self.left_panel_button.setText("HideLeftPanel")
            self._left_panel = True


    def reset_timer_button_clicked(self):
        pass

    def save_geotiff_button_clicked(self):
        pass

    def autonomous_state_button_clicked(self):
        if self._autonomous:
            self.teleop_state_button.setEnabled(True)
            self.semi_autonomous_state.setEnabled(True)
            self.search_and_rescue_button.setEnabled(True)
            self.mapping_mission_button.setEnabled(True)
            self.autonomous_state_button.setText("AutonomousState")
            self._autonomous = False
        else :
            self._GuiStateClient.transition_to_state(1)
            self.teleop_state_button.setEnabled(False)
            self.semi_autonomous_state.setEnabled(False)
            self.search_and_rescue_button.setEnabled(False)
            self.mapping_mission_button.setEnabled(False)
            self.autonomous_state_button.setText("NonAutonomousState")
            self._autonomous = True


    def teleop_state_button_clicked(self):
        self._GuiStateClient.transition_to_state(7)

    def semi_autonomous_state_clicked(self):
        self._GuiStateClient.transition_to_state(6)

    def search_and_rescue_button_clicked(self):
        self._GuiStateClient.transition_to_state(1)

    def mapping_mission_button_clicked(self):
        self._GuiStateClient.transition_to_state(2)

    #The checkboxes slots
    def sonars_checked(self):

        self._sonarsChecked = self.sonarsCheckBox.isChecked()

    def battery_checked(self):

        self._batteryChecked = self.batteryCheckBox.isChecked()

    def temp_checked(self):

        self._tempChecked = self.tempCheckBox.isChecked()

    def co2_checked(self):

        self._co2Checked = self.co2CheckBox.isChecked()

    def show_all(self):
        self._showAllChecked = self.showAllCheckBox.isChecked()

    #Method called when the Widget is terminated
    def shutdown(self):
        self.world_model_info.stop_monitoring()
        self.score_info.stop_monitoring()
        self._timer_refresh_widget.stop()
        self.ValidateVictimActionServer.shutdown()
        self._GuiStateClient.shutdown()
