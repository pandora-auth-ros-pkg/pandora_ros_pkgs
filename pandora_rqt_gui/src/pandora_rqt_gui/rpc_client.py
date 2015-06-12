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

from __future__ import print_function

__author__ = "Sideris Konstantinos"
__maintainer__ = "Sideris Konstantinos"
__email__ = "siderisk@auth.gr"

import os
import zmq


RPC_SERVER_PORT = '5555'


class Client(object):

    def __init__(self):
        master_uri = os.environ['ROS_MASTER_URI']
        self.rpc_ip = master_uri.split('//')[1].split(':')[0]
        self.context = zmq.Context()
        print('Connecting to the 0MQ Server...')
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect('tcp://' + self.rpc_ip + ':' + RPC_SERVER_PORT)

    def start_agent(self):
        print('Sending start command to the agent...')
        self.socket.send('agent:start')
        pid = self.socket.recv()
        print('Process started with pid: %s.' % pid)

    def stop_agent(self):
        print('Sending stop command to the agent...')
        self.socket.send('agent:stop')
        response = self.socket.recv()
        print('Received %s.' % response)
