#!/usr/bin/env python

# Generic driver for the Neato XV-11 Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
neato_driver.py is a generic driver for the Neato XV-11 Robotic Vacuum.
ROS Bindings can be found in the neato_node package.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import serial

"""
This driver has been changed from the original version in order to support
a wider range of neato models and firmware versions.

The expected responses are not hardcoded in this driver anymore.

This driver reads responses until it receives a control-z. Neato Robotics has
documented that all responses have a control-Z (^Z) at the end of the
response string: http://www.neatorobotics.com.au/programmer-s-manual
"""


class Botvac():

    def __init__(self, port="/dev/ttyUSB0", lds = False):
        self.port = serial.Serial(port,921600)
        # Storage for motor and sensor information
        self.state = {"FuelPercent": 0, "LeftWheel_PositionInMM": 0, "RightWheel_PositionInMM": 0, "LSIDEBIT": 0,
                "RSIDEBIT": 0, "LFRONTBIT": 0, "RFRONTBIT": 0, "SNSR_LEFT_WHEEL_EXTENDED":0, 
                "SNSR_RIGHT_WHEEL_EXTENDED":0, "BTN_SOFT_KEY": 0, "BTN_SCROLL_UP": 0, "BTN_START": 0, "BTN_BACK": 0, 
                "BTN_SCROLL_DOWN": 0, "AccelerometerX": 0, "AccelerometerY": 0, "AccelerometerZ": 0, 
                "MagSensorLeft": 0, "MagSensorRight": 0, "WallSensor": 0, "DropSensorLeft": 0, "DropSensorRight": 0}
        self.stop_state = True
        
        self.base_width = 248    # millimeters
        self.max_speed = 100     # millimeters/second
        self.crtl_z = chr(26)
        # turn things on or off
        self.port.flushInput()
        self.port.write("\n")
        self.setTestMode("on")
        self.setWiFi("off")
        if lds:
            self.setLDS("on")
        else:
            self.setLDS("off")

    def exit(self):
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("\n")
        self.setLDS("off")
        self.setTestMode("off")

    def setTestMode(self, value):
        """ Turn test mode on/off. """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("testmode " + value + "\n")
        self.readResponseString()

    def setLDS(self, value):
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("setldsrotation " + value + "\n")
        self.readResponseString()

    def setWiFi(self, value):
        """ Turn WiFi on/off. """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("setUserSettings Wifi " + value + "\n")
        self.readResponseString()

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("getldsscan\n")
        response = self.readResponseString()
        return response

    def readResponseString(self):
        """ Returns the entire response from neato in one string. """
        response = str()
        self.port.timeout = 0.03
        while True:
            try:
                buf = self.port.read(1024)
            except:
                return ""
            if len(buf) == 0:
                self.port.timeout *= 2
            else:
                response += buf
                if buf[len(buf)-1] == self.crtl_z:
                    break
        self.port.timeout = None
        return response

    def getScanRanges(self):
        """ Read values of a scan -- call requestScan first! """
        ranges = list()
        response = self.requestScan()
        for line in response.splitlines():
            #print line
            vals = line.split(",")
            # vals[[0] angle, vals[1] range, vals[2] intensity, vals[3] error code
            if len(vals) >= 2 and vals[0].isdigit() and vals[1].isdigit():
                ranges.append(int(vals[1])/1000.0)
        # sanity check
        if len(ranges) != 360:
            return []
        return ranges

    def setMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        #This is a work-around for a bug in the Neato API. The bug is that the
        #robot won't stop instantly if a 0-velocity command is sent - the robot
        #could continue moving for up to a second. To work around this bug, the
        #first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then, 
        #the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if (not self.stop_state):
                self.stop_state = True
                l = 1
                r = 1
                s = 1
        else:
            self.stop_state = False
        self.port.write("setmotor lwheeldist "+str(int(l))+" rwheeldist "+str(int(r))+" speed "+str(int(s))+"\n")

    def readResponseAndUpdateState(self):
        """ Read neato's response and update self.state dictionary.
            Call this function only after sending a command. """
        response = self.readResponseString()
        for line in response.splitlines():
            vals = line.split(",")
            if len(vals) >= 2 and vals[0].replace('_', '').isalpha() and vals[1].isdigit():
                self.state[vals[0]] = int(vals[1])

    def getMotors(self):
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("getmotors\n")
        self.readResponseAndUpdateState()
        return [self.state["LeftWheel_PositionInMM"], self.state["RightWheel_PositionInMM"]]

    def getAnalogSensors(self):
        """ Update values for analog sensors in the self.state dictionary. """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("getanalogsensors\n")
        response = self.readResponseString()
        for line in response.splitlines():
            vals = line.split(",")
            if len(vals) >= 3 and vals[0].replace('_', '').isalpha() and vals[2].replace('-', '').isdigit():
                self.state[vals[0]] = int(vals[2])
        return [self.state["AccelerometerX"], self.state["AccelerometerY"], self.state["AccelerometerZ"],
        self.state["MagSensorLeft"], self.state["MagSensorRight"], self.state["WallSensor"],
        self.state["DropSensorLeft"], self.state["DropSensorRight"]]
        

    def getDigitalSensors(self):
        """ Update values for digital sensors in the self.state dictionary. """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("getdigitalsensors\n")
        self.readResponseAndUpdateState()
        return [self.state["LSIDEBIT"], self.state["RSIDEBIT"], self.state["LFRONTBIT"], self.state["RFRONTBIT"],
                self.state["SNSR_LEFT_WHEEL_EXTENDED"], self.state["SNSR_RIGHT_WHEEL_EXTENDED"]]

    def getButtons(self):
        """ Update values for digital buttons in the self.state dictionary. """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("getbuttons\n")
        self.readResponseAndUpdateState()
        return [self.state["BTN_SOFT_KEY"], self.state["BTN_SCROLL_UP"], self.state["BTN_START"], self.state["BTN_BACK"], self.state["BTN_SCROLL_DOWN"]]
    
    def getCharger(self):
        """ Update values for charger/battery related info in self.state dictionary. """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("getcharger\n")
        self.readResponseAndUpdateState()
        return self.state["FuelPercent"]

    def setBacklight(self, value):

        if value > 0:
            self.port.write("setled backlighton\n")
        else:
            self.port.write("setled backlightoff\n")
        #self.readResponseString()

    def setLED(self, value):
        """ for older Neatos """

        if value == "Green":
            self.port.write("setled ButtonGreen\n")
        if value == "Amber":
            self.port.write("setled ButtonAmber\n")
        if value == "Red":
            self.port.write("setled LEDRed\n")
        if value == "Off":
            self.port.write("setled ButtonOff\n")
        if value == "DimGreen":
            self.port.write("setled ButtonGreenDim\n")
        if value == "DimAmber":
            self.port.write("setled ButtonAmberDim\n")

    def setLED(self, led, color, status):
        """ for Botvac D5 Connected
        led "battery" supports green, yellow an red led "info" supports blue, purple an red
        status supports solid, blink, pulse, off and dimSolid or blinkFast """

        if led == "battery":
            if status == "off":
                self.port.write("setled Led12Off\n")
            else:
                if color == "green":
                    self.port.write("setled Led1" + status + "\n")
                if color == "yellow":
                    self.port.write("setled Led12" + status + "\n")
                if color == "red":
                    self.port.write("setled Led2" + status + "\n")
        elif led == "info":
            if status == "off":
                self.port.write("setled Led34Off\n")
            else:
                if color == "blue":
                    self.port.write("setled Led3" + status + "\n")
                if color == "purple":
                    self.port.write("setled Led34" + status + "\n")
                if color == "red":
                    self.port.write("setled Led4" + status + "\n")
        #else:
            # error, led not supported

    def playSound(self, soundid):
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("playsound soundid " + str(soundid) + " \n")


    def getAllCommands(self):
        """ Extract a list of commands from the help command """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("help\n")
        response = self.readResponseString()
        #print "help:\n" + response
        commands = []
        for line in response.splitlines():
            vals = line.split(" - ")
            if len(vals) >= 2:
                commands.append(vals[0])
                #print "command found: " + vals[0]
        return commands

    class CommandDescription():
        def __init__(self, name):
            self.command = name
            self.description = ""
            self.arguments = {}
    
        def addArgument(self, arg, description):
            self.arguments[arg] = description

    def getCommandDescription(self, c):
        """ Extract a list of commands from the help command """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("help " + c + "\n")
        response = self.readResponseString()
        #print "documentation of " + command + ":\n" + response
        command = Botvac.CommandDescription(c)
        current_argument = ""
        for line in response.splitlines():
            vals = line.split(" - ")
            if "    " in line and "      " not in line and len(vals) == 2:
                command.addArgument(vals[0], vals[1])
                current_argument = vals[0]
            elif "help " not in line and self.crtl_z not in line:
                if current_argument not in "":
                    command.arguments[current_argument] += " <br> " + line
                elif len(vals) ==2 and c in vals[0]:
                    command.description += "" + vals[1]
                else: 
                    command.description += " <br> " + line
        return command

