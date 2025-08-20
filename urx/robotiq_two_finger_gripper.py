#! /usr/bin/env python

"""
Python library to control Robotiq Two Finger Gripper connected to UR robot via
Python-URX

Tested using a UR5 Version CB3 and Robotiq 2-Finger Gripper Version 85

SETUP

You must install the driver first and then power on the gripper from the
gripper UI. The driver can be found here:

http://support.robotiq.com/pages/viewpage.action?pageId=5963876

FAQ

Q: Why does this class group all the commands together and run them as a single
program as opposed to running each line seperately (like most of URX)?

A: The gripper is controlled by connecting to the robot's computer (TCP/IP) and
then communicating with the gripper via a socket (127.0.0.1:63352).  The scope
of the socket is at the program level.  It will be automatically closed
whenever a program finishes.  Therefore it's important that we run all commands
as a single program.

DOCUMENTATION

- This code was developed by downloading the gripper package "DCU-1.0.10" from
  http://support.robotiq.com/pages/viewpage.action?pageId=5963876. Or more
  directly from http://support.robotiq.com/download/attachments/5963876/DCU-1.0.10.zip
- The file robotiq_2f_gripper_programs_CB3/rq_script.script was referenced to
  create this class

# WRITE VARIABLES (CAN ALSO READ)
ACT = ‘ACT’ # act : activate (1 while activated, can be reset to clear fault status)
GTO = ‘GTO’ # gto : go to (will perform go to with the actions set in pos, for, spe)
ATR = ‘ATR’ # atr : auto-release (emergency slow move)
ADR = ‘ADR’ # adr : auto-release direction (open(1) or close(0) during auto-release)
FOR = ‘FOR’ # for : force (0-255)
SPE = ‘SPE’ # spe : speed (0-255)
POS = ‘POS’ # pos : position (0-255), 0 = open
# READ VARIABLES
STA = ‘STA’ # status (0 = is reset, 1 = activating, 3 = active)
PRE = ‘PRE’ # position request (echo of last commanded position)
OBJ = ‘OBJ’ # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
FLT = ‘FLT’ # fault (0=ok, see manual for errors if not zero)
"""  # noqa

import logging
import os
import time

from urx.urscript import URScript

# Gripper Variables
ACT = "ACT"
GTO = "GTO"
ATR = "ATR"
ARD = "ARD"
FOR = "FOR"
SPE = "SPE"
OBJ = "OBJ"
STA = "STA"
FLT = "FLT"
POS = "POS"

SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 63352
SOCKET_NAME = "gripper_socket"


class RobotiqScript(URScript):

    def __init__(self,
                 socket_host=SOCKET_HOST,
                 socket_port=SOCKET_PORT,
                 socket_name=SOCKET_NAME):
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name
        super(RobotiqScript, self).__init__()

        # Reset connection to gripper
        self._socket_close(self.socket_name)
        self._socket_open(self.socket_host,
                          self.socket_port,
                          self.socket_name)

    def _import_rq_script(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        rq_script = os.path.join(dir_path, 'rq_script.script')
        with open(rq_script, 'rb') as f:
            rq_script = f.read()
            self.add_header_to_program(rq_script)

    def _rq_get_var(self, var_name, nbytes):
        self._socket_send_string("GET {}".format(var_name), self.socket_name)
        self._socket_read_byte_list(nbytes, self.socket_name)

    def _get_gripper_fault(self):
        self._rq_get_var(FLT, 2)

    def _get_gripper_object(self):
        self._rq_get_var(OBJ, 1)

    def _get_gripper_status(self):
        self._rq_get_var(STA, 1)
        
    def _get_gripper_pos(self):
        self._rq_get_var(POS, 1)

    def _set_gripper_activate(self):
        self._socket_set_var(GTO, 1, self.socket_name)

    def _set_gripper_force(self, value):
        """
        FOR is the variable
        range is 0 - 255
        0 is no force
        255 is full force
        """
        value = self._constrain_unsigned_char(value)
        self._socket_set_var(FOR, value, self.socket_name)

    def _set_gripper_position(self, value):
        """
        SPE is the variable
        range is 0 - 255
        0 is no speed
        255 is full speed
        """
        value = self._constrain_unsigned_char(value)
        self._socket_set_var(POS, value, self.socket_name)

    def _set_gripper_speed(self, value):
        """
        SPE is the variable
        range is 0 - 255
        0 is no speed
        255 is full speed
        """
        value = self._constrain_unsigned_char(value)
        self._socket_set_var(SPE, value, self.socket_name)

    def _set_robot_activate(self):
        self._socket_set_var(ACT, 1, self.socket_name)
        
    


class Robotiq_Two_Finger_Gripper(object):

    def __init__(self,
                 robot,
                 payload=0.85,
                 speed=255,
                 force=50,
                 socket_host=SOCKET_HOST,
                 socket_port=SOCKET_PORT,
                 socket_name=SOCKET_NAME):
        self.robot = robot
        self.payload = payload
        self.speed = speed
        self.force = force
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name
        self.logger = logging.getLogger(u"robotiq")

        # Send the start script to init and close gripper
        urscript = self._get_new_urscript()

        # Initialize the gripper
        urscript._set_robot_activate()
        urscript._set_gripper_activate()

        # Wait on activation to avoid USB conflicts
        urscript._sleep(0.1)
        urscript._set_gripper_position(255)

        self.robot.send_program(urscript())
        
        time.sleep(2.0)
        
        
    def _set_analog_out_to_var(self, var):
        # Set analog out to variable's value 
        urscript = self._get_new_urscript()
        urscript.add_line_to_program('rq_var = socket_get_var("{}","{}")'.format(var, self.socket_name))
        urscript._sync()
        urscript.add_line_to_program('set_standard_analog_out(0, rq_var / 255.0)')
        urscript.add_line_to_program('textmsg(rq_var / 255.0)')
        self.robot.send_program(urscript())
        
        
    def _get_var(self, var):
        self._set_analog_out_to_var(var)
        time.sleep(0.2)
        return int(self.robot.secmon.get_analog_out(0)/10.0*255)


    def _get_new_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = RobotiqScript(socket_host=self.socket_host,
                                 socket_port=self.socket_port,
                                 socket_name=self.socket_name)

        # Set input and output voltage ranges
        urscript._set_analog_inputrange(0, 0)
        urscript._set_analog_inputrange(1, 0)
        urscript._set_analog_inputrange(2, 0)
        urscript._set_analog_inputrange(3, 0)
        urscript._set_analog_outputdomain(0, 1)
        urscript._set_analog_outputdomain(1, 1)
        urscript._set_tool_voltage(0)
        urscript._set_runstate_outputs()

        # Set payload, speed and force
        urscript._set_payload(self.payload)
        urscript._set_gripper_speed(self.speed)
        urscript._set_gripper_force(self.force)

        return urscript


    def gripper_action(self, value):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        urscript = self._get_new_urscript()

        # Move to the position
        sleep = 2.0
        urscript._set_gripper_position(value)
        urscript._sleep(sleep)

        # Send the script
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(sleep)

    def open_gripper(self):
        self.gripper_action(0)

    def close_gripper(self):
        self.gripper_action(255)
        
    def check_obj_grasp(self):
    # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
        return self._get_var(OBJ)
        
    def get_pos(self):
        return self._get_var(POS)
        
