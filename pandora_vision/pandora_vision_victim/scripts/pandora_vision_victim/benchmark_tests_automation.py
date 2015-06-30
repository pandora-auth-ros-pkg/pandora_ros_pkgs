#!/usr/bin/env python
"""Automates the selection of a suitable classification model by executing a set
of benchmark tests for a set of models.
"""

PKG = "pandora_vision_victim"

import os
import sys
import shutil
import readline
import subprocess
import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import rospkg

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
__author__ = "Kofinas Miltiadis"
__maintainer__ = "Kofinas Miltiadis"
__email__ = "mkofinas@gmail.com"

def find_ros_workspace(current_package_path):
    """Returns the the path to the ROS workspace that this package belongs.
    """
    ros_package_paths = str(subprocess.check_output("echo $ROS_PACKAGE_PATH",
                            shell=True))
    ros_package_paths = ros_package_paths.split(":")

    for ros_path in ros_package_paths:
        ros_path = ros_path.strip("\n")
        if ros_path in current_package_path:
            return os.path.dirname(ros_path)
    else:
        print "Error! This package doesn't seem to belong in any of the paths "\
              "inside ROS_PACKAGE_PATH. Exiting!"
        sys.exit()


def help_details():
    """Prints details about the desired format of each of the folders used for
    benchmark tests.
    """
    print "Each folder should contain a classifier model, possibly and 'xml' "\
          "or a 'yaml' file. Additionaly, "\
          "if feature normalization is to be performed, normalization "\
          "parameters files should exist. Finally, a file containing the "\
          "training parameters should be present (.yaml). The last file is "\
          "presumed to contain the keyword 'params' in its name."

def execute_benchmark_tests():
    """Executes a set of benchmark tests for a set of classification models.
    """
    # Set the auto completion scheme
    readline.set_completer_delims(" \t")
    readline.parse_and_bind("tab:complete")

    print "Type the absolute path to the classifier models:"
    print "e.g. /home/user/foo"

    help_details()

    classifiers_path = raw_input("-->")

    if not os.path.isdir(classifiers_path):
        print "The specified path is not valid. Exiting!"
        return None

    package_path = rospkg.RosPack().get_path(PKG)

    ros_workspace_path = find_ros_workspace(package_path)

    package_data_path = os.path.join(package_path, "data")
    package_config_path = os.path.join(package_path, "config")

    for folder in os.listdir(classifiers_path):
        print "Starting new test procedure!"
        folder_path = os.path.join(classifiers_path, folder)

        print "Copying necessary files to the package folder!"
        for data_file in os.listdir(folder_path):
            if "params" in data_file:
                shutil.copyfile(os.path.join(folder_path, data_file),
                                os.path.join(package_config_path, data_file))
            else:
                shutil.copyfile(os.path.join(folder_path, data_file),
                                os.path.join(package_data_path, data_file))

        os.chdir(os.path.abspath(ros_workspace_path))

        print "Starting Test! \n"
        command = "rostest pandora_vision_victim victim_benchmark_test.launch -t"
        subprocess.call(command, shell=True)
        # Sleep for a while until the process is fully finished.
        rospy.sleep(1)
        print "Finished one test"
        shutil.copyfile(os.path.join(package_data_path,
                                     "benchmark_results.txt"),
                                     os.path.join(folder_path,
                                     "benchmark_results.txt"))
        os.rename(os.path.join(folder_path, "benchmark_results.txt"),
                  os.path.join(folder_path, folder + ".txt"))
    print "Process is finished successfully!"


if __name__ == "__main__":
    execute_benchmark_tests()
