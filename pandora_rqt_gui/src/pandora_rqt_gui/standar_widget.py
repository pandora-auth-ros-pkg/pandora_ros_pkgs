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
import dynamic_reconfigure.client

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Slot, QTime
from python_qt_binding.QtGui import QWidget, QPixmap

from std_msgs.msg import Int32
from pandora_data_fusion_msgs.msg import WorldModelMsg

from .widget_info import WidgetInfo
from .victim_found_server import ValidateVictimActionServer
from .probability_info import ProbabilityInfoWidget
from .console import Console
from .gui_state_client import GuiStateClient
from .save_mission_client import SaveMissionClient

world_model_info_topic = '/data_fusion/world_model'
robocup_score_topic = 'data_fusion/robocup_score'
validate_victim_service_name = '/gui/validate_victim'


class StandarWidget(QWidget):
    """
    StandarWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin=None):

        super(StandarWidget, self).__init__()
        #Load Ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('pandora_rqt_gui'),
            'resources', 'StandarWidget.ui')
        loadUi(ui_file, self)

        self.Console_ = Console()
        self.ConsoleWidget_ = self.Console_._widget

        self.ProbabilityInfoWidget_ = ProbabilityInfoWidget(self)
        self.GuiStateClient_ = GuiStateClient()

        #Add Console in the 2,1 position of InternalGrid
        self.internal_grid.addWidget(self.ConsoleWidget_, 2, 1)

        #use full ABSOLUTE path to the image, not relative
        self.image.setPixmap(QPixmap(os.path.join(
            rp.get_path('pandora_rqt_gui'),
            'images', 'pandora_logo.jpeg')))

        # the ValidateVictimActionServer is used called when a victim is found
        self.ValidateVictimActionServer_ = ValidateVictimActionServer(
            validate_victim_service_name)
        # dynamic_reconfigure client is used to change the parameters
        #~ self.dynamic_reconfigure_client = dynamic_reconfigure.client.Client("agent")

        self.dynamic_reconfigure_client = None

        #The SaveMissionClient is used to save the mission geotiff
        self.SaveMissionClient = SaveMissionClient()

        #Subscribe the score the world_model_info and Info
        self.score_info = WidgetInfo(robocup_score_topic, Int32)
        self.world_model_info = WidgetInfo(world_model_info_topic, WorldModelMsg)
        self.victim_info = []

        self.timer_started = False

        #connecting the radioButtons
        self.yellow_arena_radio_button.clicked.connect(self.yellow_arena_radio_button_clicked)
        self.yellow_black_arena_radio_button.clicked.connect(self.yellow_black_arena_radio_button_clicked)
        self.mapping_mission_radio_button.clicked.connect(self.mapping_mission_radio_button_clicked)

        #Connecting the Buttons
        self.go_button.clicked.connect(self.go_button_clicked)
        self.stop_button.clicked.connect(self.stop_button_clicked)
        self.confirm_button.clicked.connect(self.confirm_victim_clicked)
        self.decline_button.clicked.connect(self.decline_button_clicked)
        self.left_panel_button.clicked.connect(self.left_panel_button_clicked)
        self.console_options_button.clicked.connect(self.console_options_button_clicked)
        self.save_geotiff_button.clicked.connect(self.save_geotiff_button_clicked)
        self.teleop_state_button.clicked.connect(self.teleop_state_button_clicked)
        self.sensor_test_state_button.clicked.connect(self.sensor_test_state_button_clicked)
        self.semi_autonomous_state_button.clicked.connect(self.sensor_test_state_button_clicked)
        self.terminate_button.clicked.connect(self.terminate_button_clicked)

        #Connecting the CheckBoxes
        self.temp_checkbox.stateChanged.connect(self.temp_checked)
        self.co2_checkbox.stateChanged.connect(self.co2_checked)
        self.sonars_checkbox.stateChanged.connect(self.sonars_checked)
        self.battery_checkbox.stateChanged.connect(self.battery_checked)
        self.show_all_checkbox.stateChanged.connect(self.show_all)
        self.terminate_check_box.stateChanged.connect(self.terminate_checked)

        #In the Beggining all the checkboxes are unchecked(used for MainWidget)
        self.temp_checked = False
        self.battery_checked = False
        self.sonars_checked = False
        self.co2_checked = False
        self.show_all_checked = True

        #The left panel is visible
        self.left_panel = True

        # Resize at first
        self.resize = True

        #Show the console options at first
        self.console_show_options = True

        # Refresh timer
        self.timer_refresh_widget = QTimer(self)
        self.timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):

        self.score_info.start_monitoring()
        self.world_model_info.start_monitoring()
        self.timer_refresh_widget.start(1000)

    def enable_victim_found_options(self):

        self.confirm_button.setEnabled(True)
        self.decline_button.setEnabled(True)
        self.victimx.setEnabled(True)
        self.victimy.setEnabled(True)
        self.sensorID.setEnabled(True)
        self.probability.setEnabled(True)
        self.setvictim_info()
        self.ProbabilityInfoWidget_.show()
        self.internal_grid.addWidget(self.ProbabilityInfoWidget_, 1, 0)
        self.ProbabilityInfoWidget_.start()

    def disable_victim_found_options(self):

        self.confirm_button.setEnabled(False)
        self.decline_button.setEnabled(False)
        self.victimx.setEnabled(False)
        self.victimy.setEnabled(False)
        self.sensorID.setEnabled(False)
        self.probability.setEnabled(False)
        self.internal_grid.removeWidget(self.ProbabilityInfoWidget_)
        self.ProbabilityInfoWidget_.shutdown()
        self.ProbabilityInfoWidget_.close()

    def setvictim_info(self):

        self.victimx.setText(" Victim X Position " + str(self.victim_info[0]))
        self.victimy.setText(" Victim Y Position " + str(self.victim_info[1]))
        sensors = ""
        for sensor in self.victim_info[4]:
            sensors = sensors + (sensor) + " "

        self.sensorID.setText(" Sensor IDs " + "".join(sensors))
        self.probability.setText(" Probability " + str(self.victim_info[3]))

    # Refreshing the topics
    @Slot()
    def refresh_topics(self):

        self.state.setText(self.GuiStateClient_.get_state())
        if self.resize:
            self.ConsoleWidget_._handle_column_resize_clicked()
            self.resize = False

        if self.score_info.last_message is not None:
            self.score.display(self.score_info.last_message.data)

        if self.world_model_info.last_message is not None:
            victims_number = 0

            for victim in self.world_model_info.last_message.visitedVictims:

                if victim.valid:
                    victims_number = victims_number + 1
            self.victims_found.display(victims_number)

        if self.timer_started:
            self.time_ = self.time_.addSecs(-1)
            self.timer.setTime(self.time_)

        # Enable the victim found Options if it is found
        if self.ValidateVictimActionServer_.victim_found:
            self.victim_info = self.ValidateVictimActionServer_.victim_info
            self.enable_victim_found_options()

    #Start the timer
    def go_button_clicked(self):

        self.timer_started = True
        self.go_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.yellow_arena_radio_button.setEnabled(False)
        self.yellow_black_arena_radio_button.setEnabled(False)
        self.mapping_mission_radio_button.setEnabled(False)
        self.sensor_test_state_button.setEnabled(False)
        self.time_ = (self.timer.time())
        self.timer.setTime(self.time_)
        self.GuiStateClient_.transition_to_state(1)
        self.ConsoleWidget_._handle_column_resize_clicked()

    #Stop the timer and The robot
    def stop_button_clicked(self):
        self.ConsoleWidget_._handle_column_resize_clicked()
        self.timer_started = False
        self.go_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.GuiStateClient_.transition_to_state(0)

    def decline_button_clicked(self):

        self.ValidateVictimActionServer_.victim_valid = False
        self.disable_victim_found_options()
        self.ValidateVictimActionServer_.operator_responded = True
        self.ValidateVictimActionServer_.victim_found = False

    def confirm_victim_clicked(self):
        self.ValidateVictimActionServer_.victim_valid = True
        self.ValidateVictimActionServer_.operator_responded = True
        self.disable_victim_found_options()
        self.ValidateVictimActionServer_.victim_found = False

    def left_panel_button_clicked(self):

        if self.left_panel:
            self.image.close()
            self.victim_info_2.close()
            self.left_panel_button.setText("Show Left Panel")
            self.left_panel = False
        else:
            self.image.show()
            self.victim_info_2.show()
            self.left_panel_button.setText("Hide Left Panel")
            self.left_panel = True

    def console_options_button_clicked(self):
        if self.console_show_options:
            self.console_options_button.setText("Console Options")
            self.ConsoleWidget_.exclude_group_box.close()
            self.ConsoleWidget_.highlight_group_box.close()
            self.console_show_options = False
        else:
            self.console_options_button.setText("Hide Console Options")
            self.ConsoleWidget_.exclude_group_box.show()
            self.ConsoleWidget_.highlight_group_box.show()
            self.console_show_options = True

    def save_geotiff_button_clicked(self):
            mission_name = self.mission_name_text.toPlainText()
            self.SaveMissionClient.save_mission_client(mission_name)

    def teleop_state_button_clicked(self):
        self.yellow_arena_radio_button.setEnabled(True)
        self.yellow_black_arena_radio_button.setEnabled(True)
        self.mapping_mission_radio_button.setEnabled(True)
        self.go_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.GuiStateClient_.transition_to_state(6)

    def semi_autonomous_state_clicked(self):
        self.yellow_arena_radio_button.setEnabled(True)
        self.yellow_black_arena_radio_button.setEnabled(True)
        self.mapping_mission_radio_button.setEnabled(True)
        self.go_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.GuiStateClient_.transition_to_state(5)

    def sensor_test_state_button_clicked(self):
        self.GuiStateClient_.transition_to_state(7)

    def terminate_button_clicked(self):
        self.timer_started = False
        self.yellow_arena_radio_button.setEnabled(False)
        self.yellow_black_arena_radio_button.setEnabled(False)
        self.mapping_mission_radio_button.setEnabled(False)
        self.go_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.teleop_state_button.setEnabled(False)
        self.semi_autonomous_state_button.setEnabled(False)
        self.sensor_test_state_button.setEnabled(False)
        self.GuiStateClient_.transition_to_state(9)

    def yellow_arena_radio_button_clicked(self):
        if self.dynamic_reconfigure_client is not None:
            self.dynamic_reconfigure_client.update_configuration(
                {"arenaType": 0})

    def yellow_black_arena_radio_button_clicked(self):
        if self.dynamic_reconfigure_client is not None:
            self.dynamic_reconfigure_client.update_configuration(
                {"arenaType": 1})

    def mapping_mission_radio_button_clicked(self):
        if self.dynamic_reconfigure_client is not None:
            self.dynamic_reconfigure_client.update_configuration(
                {"arenaType": 2})

    #The _checkboxes slots
    def sonars_checked(self):

        self.sonars_checked = self.sonars_checkbox.isChecked()

    def battery_checked(self):

        self.battery_checked = self.battery_checkbox.isChecked()

    def temp_checked(self):

        self.temp_checked = self.temp_checkbox.isChecked()

    def co2_checked(self):

        self.co2_checked = self.co2_checkbox.isChecked()

    def show_all(self):
        self.show_all_checked = self.show_all_checkbox.isChecked()

    def terminate_checked(self):
        self.terminate_button.setEnabled(self.terminate_check_box.isChecked())

    #Method called when the Widget is terminated
    def shutdown(self):
        self.world_model_info.stop_monitoring()
        self.score_info.stop_monitoring()
        self.timer_refresh_widget.stop()
        self.ValidateVictimActionServer_.shutdown()
        self.GuiStateClient_.shutdown()
