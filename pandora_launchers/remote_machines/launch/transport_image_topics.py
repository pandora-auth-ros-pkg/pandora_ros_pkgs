#!/usr/bin/env python

__author__ = "Konstantinos Panayiotou"
__maintainer__ = "Konstantinos Panayiotou"
__email__ = "klpanagi@gmail.com"
__version__ = "0.0.0"


import argparse
import json
import sys
import os  # System calls
import subprocess  # System calls as subprocesses
import re  # Regular Expressions
__path__ = os.path.dirname(os.path.realpath(__file__))


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


## @brief Execute a shell command as a subprocess
#  @param command Command to execute as subprocess
#  @return
def shell(command):
    cmd = command.split(" ")
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = p.communicate()
    if stderr != None and stderr != '':
        print bcolors.FAIL + "Error on command execution: [%s]" % command
        print bcolors.UNDERLINE + "Error ---> %s" % stderr +  bcolors.ENDC
        return 0
    return stdout


## @brief Execute a shell command as a subprocess -- Detatched
#  @param command Command to execute as a detatched subprocess
#  @return Bool True if command executed succesfully. False otherwhise
def shell_detatched(command):
    cmd = command.split(" ")
    try:
        # Execute command
        p = subprocess.Popen(cmd)
    except:
        e = sysexc_info()[0]
        # On failed to execute command
        print bcolors.FAIL + "Error on command execution: [%s]" % command
        print bcolors.UNDERLINE + "Error ---> %s" % e
        return 0
    return 1 # On success execution


## @brief Extracts Topic-Name and Topic-Type from given publisher
#  @param publisher Publisher to extract info from
#  @return List. Containes 'name' and 'type' parameters
def extract_topic_info(publisher):
    temp = publisher.split(' ')
    topic = {'name': temp[0], 'type': temp[1]}
    return topic


## @brief TSearch for active publishers from a given node
#  @param node Node to search for publishers
#  @return Found publishers
def active_publishers(node):
    publishers = []
    cmd = "rosnode info %s" % node
    stdout = shell(cmd)
    publications = stdout.split('Publications: \n')[1].split('Subscriptions')[0]
    publications = publications.split('\n\n')[0]
    publications = publications.split('\n')

    print bcolors.OKBLUE + "Active publishers on node [%s]: " % node + \
        bcolors.ENDC

    for pub in publications:
        pub_clean = pub.split(' * ')[1]
        publishers.append(pub_clean)
        print pub_clean

    return publishers


## @brief Tries to match a machine namespace from a given topic name
#  @return Returns True if RegExpr mathced with success.
def match_topic_machine(machine, topic):
  expr = r'/%s' % machine
  res = re.search( expr, topic['name'] )
  if res is not None:
      print bcolors.OKGREEN + "Matched remote machine [%s] topic [%s]: " \
        % (machine, topic['name']) + bcolors.ENDC
      return True
  else:
      return False


## @brief Checks for sensor_msgs/Image topic type
#  @param topic Topic to check for.
#  @return Bool. True on found. False otherwise
def is_image_topic(topic):
    image_type = 'sensor_msgs/Image'
    expr = r'%s' % image_type
    res = re.search( expr, topic['type'] )
    if res is not None:
      print bcolors.OKGREEN + "Found [%s] topic type ---> " \
        % image_type + bcolors.UNDERLINE + "{%s}" % topic['name'] + \
        bcolors.ENDC
      return True
    else:
      return False


## @brief Search for active nodes on remote machone
#  @param machine The machine nodes to search for.
#  @return Nodes found on remote machine
def active_nodes(machine):
    cmd = "rosnode machine %s" % machine
    ret = shell(cmd)
    if ret == 0 or ret == '\n':
        print bcolors.FAIL + "No active nodes on machine [%s]" % machine
        sys.exit(1)

    nodes = ret.split('\n')
    nodes.remove('')
    print bcolors.OKBLUE + bcolors.UNDERLINE + "Active nodes on machine [%s]: " \
        % machine + bcolors.ENDC
    for node in nodes:
        print node
    return nodes


## @brief Republish a topic using image_transport
#   Use this re republish nodes running on remote machine, on local machine.
#  @param in_topic Topic to transport (e.g Remote machine topic)
#  @param out_topic Transported topic (e.g Local machine topic)
def republish(in_topic, out_topic):
    cmd = "rosrun image_transport republish raw in:=%s raw out:=%s" \
        % (in_topic, out_topic)
    #print cmd
    print bcolors.HEADER + "Republishing topic " + bcolors.UNDERLINE + \
        "%s" % in_topic + bcolors.ENDC + bcolors.WARNING + " ----> " + \
        bcolors.ENDC + bcolors.HEADER + bcolors.UNDERLINE + "%s" % out_topic + \
        bcolors.ENDC

    shell_detatched(cmd)


def run(machine):
    nodes = active_nodes(machine)
    if nodes == None:
        sys.exit(1)
    for node in nodes:
        publishers  = active_publishers(node)
        for pub in publishers:
            topic = extract_topic_info(pub)
            if is_image_topic(topic):
                in_topic = topic['name']
                expr = '/%s' % machine
                out_topic = topic['name'].split(expr)[1]
                republish(in_topic, out_topic)


def main():
    # ----------------Initialize console args parser------------------------- #
    parser = argparse.ArgumentParser(description=  'Transport' + \
        'sensor_msgs/Image topics running on machine=<machine> locally.' + \
        'Example of use is to transport topics on a remote machinee locally' + \
        'in order to decrease processing power on remote machine when' +
        'used by multiple subscribers.')

    parser.add_argument('-i','--machine', help='Machine', dest='machine', \
        action='store', nargs='+', type=str)
    args =  parser.parse_args( ) # Parse console arguments.

    if args.machine == None:
        # Did not set machine
        print bcolors.FAIL + "Machine name not specified!!" + bcolors.ENDC
        sys.exit(1)

    machine = args.machine[0]
    #machine = "rpi2"
    run(machine)
    sys.exit(0)


if __name__ == "__main__":
    main()
