#!/usr/bin/env python

import rospy, pypandoc

from neato_driver.neato_driver import Botvac

def commonDocumentation():
    doc = "## Communication through the USB port\n"
    doc += "\n"
    doc += "Remove dust filter and plug in a USB cable to the USB port and to \n"
    doc += "your computer. Then you can use a terminal to communicate with the \n"
    doc += "robot. For example you can use screen, with the device as argument. \n"
    doc += "The device can be found in /dev folder, usually it is ttyACM0, \n"
    doc += "sometimes it is ttyACM1. You can type ''screen /dev/ttyACM0'' to \n"
    doc += "start the communication. Type the first command, for example help, \n"
    doc += "and press Enter.  \n"
    doc += "\n"
    doc += "## Command syntax\n"
    doc += "\n"
    doc += "Every commant starts with its name, followed by arguments. The \n"
    doc += "arguments are either flags or key value pairs. The arguments are \n"
    doc += "seperated by spaces, keys and values too. Commands and arguments \n"
    doc += "are not case insensitive, even partial commands are supported.\n"
    doc += "\n"
    doc += "## Response syntax\n"
    doc += "\n"
    doc += "Command responses are in csv (comma seperated values), beginning \n"
    doc += "with a title line. Each response ends with control-z\n"
    doc += "\n"
    doc += "## List of Commands\n"
    doc += "\n"
    doc += "The following commands are supported by the robot:\n"
    doc += "\n"
    return doc;

def readCommandsFromNeato():
    port = rospy.get_param('~port', "/dev/ttyACM0")
    robot = Botvac(port)
    doc = ""
    commands = robot.getAllCommands()
    for c in commands:
        com = robot.getCommandDescription(c)
        doc += "### " + com.command + "\n\n"
        doc += "**Description:** " + com.description  + "\n\n"
        if len(com.arguments) > 0:
            doc += "**Options:** \n\n| Flag | Description |\n|------|-------------|\n"
            for arg in com.arguments:
                doc += "| " + arg + " | "  + com.arguments[arg] + " |\n"
            doc += "\n\n"
    robot.exit()
    return doc

if __name__ == "__main__":    
    documentation = "# Manual for Neato's serial port interface\n"
    documentation += commonDocumentation()
    #documentation += readCommandsFromNeato()
    format = rospy.get_param('~format', "md")
    filename = rospy.get_param('~filename', "output.md")
    output = pypandoc.convert_text(documentation, format, format='md', outputfile=filename)
