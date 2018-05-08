#!/usr/bin/env python

# ROS node for the Neato Robot Vacuum
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
ROS node for Neato robot vacuums.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import roslib; roslib.load_manifest("neato_node")
import rospy
import sys
import traceback
from math import sin,cos

from sensor_msgs.msg import LaserScan, Range
from std_srvs.srv import SetBool, SetBoolResponse
from neato_node.msg import Button, Sensor, Movement
from neato_node.srv import SetLed, SetLedResponse, PlaySound, PlaySoundResponse
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from neato_driver.neato_driver import Botvac

class NeatoNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('neato')

        self.port = rospy.get_param('~port', "/dev/ttyUSB0")
        self.lds = rospy.get_param('~lds', True)
        rospy.loginfo("Using port: %s" % self.port)

        self.robot = Botvac(self.port, self.lds)

        rospy.Subscriber("cmd_dist", Movement, self.cmdMovementCb)
        self.scanPub = rospy.Publisher('base_scan', LaserScan, queue_size=1)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.buttonPub = rospy.Publisher('button', Button, queue_size=1)
        self.sensorPub = rospy.Publisher('sensor', Sensor, queue_size=1)
        self.accelerationPub = rospy.Publisher('acceleration', Vector3Stamped, queue_size=1)
        self.wallPub = rospy.Publisher('wall', Range, queue_size=1)
        self.drop_leftPub = rospy.Publisher('drop_left', Range, queue_size=1)
        self.drop_rightPub = rospy.Publisher('drop_right', Range, queue_size=1)
        self.magneticPub = rospy.Publisher('magnetic', Sensor, queue_size=1)
        self.odomBroadcaster = TransformBroadcaster()

        rospy.Service('set_info_led', SetLed, self.setInfoLed)
        rospy.Service('play_sound', PlaySound, self.playSound)
        rospy.Service('set_lds', SetBool, self.setLDS)

        self.cmd_vel = 0
        self.cmd_dist = [0, 0]
        self.update_movement = False
        self.encoders = [0, 0]

    def spin(self):
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id', 'base_laser_link')
        scan = LaserScan(header=rospy.Header(frame_id=scan_link))
        scan.angle_min = -3.13
        scan.angle_max = +3.13
        scan.angle_increment = 0.017437326
        scan.range_min = 0.020
        scan.range_max = 5.0

        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')
        self.odomPub.publish(odom)

        button = Button()
        sensor = Sensor()
        magnetic = Sensor()
        range_sensor = Range()
        range_sensor.radiation_type = 1
        #range_sensor.field_of_view = 
        range_sensor.min_range = 0.0
        range_sensor.max_range = 0.255
        acceleration = Vector3Stamped()
        self.robot.setBacklight(1)
        self.robot.setLED("info", "blue", "solid")
        # main loop of driver
        r = rospy.Rate(5)
        loop_counter = 0
        try: 
            while not rospy.is_shutdown():

                if loop_counter == 4:
                    self.set_battery_status()
                    self.publish_scan(scan)
                    loop_counter = 0
                else:
                    loop_counter += 1

                self.publish_odom(odom)
                self.publish_buttons(button)
                drop_left, drop_right, ml, mr = self.publish_analog(acceleration, range_sensor, magnetic)
                lw, rw, lsb, rsb, lfb, rfb = self.publish_digital(sensor)

                # send updated movement commands
                if self.violate_safety_constraints(drop_left, drop_right, ml, mr, lw, rw, lsb, rsb, lfb, rfb):
                    self.robot.setMotors(0, 0, 0)
                    self.cmd_vel = 0
                elif self.update_movement:
                    self.robot.setMotors(self.cmd_dist[0], self.cmd_dist[1], self.cmd_vel)
                    # reset update flag 
                    self.update_movement = False
              # wait, then do it again
                r.sleep()

            # shut down
            self.robot.setMotors(0,0,0)
            self.robot.setBacklight(0)
            self.robot.setLED("Battery", "Green", "Off")
            self.robot.setLED("Info", "Blue", "Off")
            self.robot.setLDS("off")
            self.robot.setTestMode("off")
        except:
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)
            self.robot.setMotors(0,0,0)
            self.robot.setBacklight(0)
            self.robot.setLED("Battery", "Green", "Off")
            self.robot.setLED("Info", "Red", "Solid")
            self.robot.setLDS("off")
            self.robot.setTestMode("off")

    def cmdVelCb(self,req):
        x = req.linear.x * 1000
        th = req.angular.z * (self.robot.base_width/2)
        k = max(abs(x-th),abs(x+th))
        # sending commands higher than max speed will fail
        if k > self.robot.max_speed:
            x = x*self.robot.max_speed/k; th = th*self.robot.max_speed/k
        self.cmd_vel = [int(x-th), int(x+th)]

    def  cmdMovementCb(self,req):
        k = req.vel
        # sending commands higher than max speed will fail
        if k > self.robot.max_speed:
            k = self.robot.max_speed
            rospy.logwarn("You have set the speed to more than the maximum speed of the neato. For safety reasons it is set to %d", self.robot.max_speed)
        self.cmd_vel = k
        self.cmd_dist = [req.x_dist*1000, req.y_dist*1000]
        self.update_movement = True

    def publish_odom(self, odom):
        # get motor encoder values
        left, right = self.robot.getMotors()

        d_left = (left - self.encoders[0])/1000.0
        d_right = (right - self.encoders[1])/1000.0
        self.encoders = [left, right]

        dx = (d_left+d_right)/2
        dth = (d_right-d_left)/(self.robot.base_width/1000.0)

        x = cos(dth)*dx
        y = -sin(dth)*dx
        self.x += cos(self.th)*x - sin(self.th)*y
        self.y += sin(self.th)*x + cos(self.th)*y
        self.th += dth

        # prepare tf from base_link to odom
        quaternion = Quaternion()
        quaternion.z = sin(self.th/2.0)
        quaternion.w = cos(self.th/2.0)

        dt = (odom.header.stamp - rospy.Time.now()).secs

        # prepare odometry
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = dx/dt
        odom.twist.twist.angular.z = dth/dt

        # publish everything
        self.odomPub.publish(odom)
        self.odomBroadcaster.sendTransform((self.x, self.y, 0), (quaternion.x, quaternion.y, quaternion.z,
                                                                         quaternion.w), rospy.Time.now(), "base_link", "odom")
    def publish_scan(self, scan):
        scan.header.stamp = rospy.Time.now()
        scan.ranges = self.robot.getScanRanges()
        self.scanPub.publish(scan)

    def publish_analog(self, acceleration, range_sensor, magnetic):
        # analog sensors
        ax, ay, az, ml, mr, wall, drop_left, drop_right = self.robot.getAnalogSensors()
        acceleration.header.stamp = rospy.Time.now()
        # convert mG to m/s^2
        acceleration.vector.x = ax * 9.80665/1000.0
        acceleration.vector.y = ay * 9.80665/1000.0
        acceleration.vector.z = az * 9.80665/1000.0
        range_sensor.header.stamp = rospy.Time.now()

        self.accelerationPub.publish(acceleration)

        range_sensor.range = wall / 1000.0
        self.wallPub.publish(range_sensor)
        range_sensor.range = drop_left / 1000.0
        self.drop_leftPub.publish(range_sensor)
        range_sensor.range = drop_right / 1000.0
        self.drop_rightPub.publish(range_sensor)

        magnetic_enum = ("Left_Sensor", "Right_Sensor")
        for idx, val in enumerate((ml, mr)):
            magnetic.value = val
            magnetic.name = magnetic_enum[idx]
            self.magneticPub.publish(magnetic)
        return drop_left, drop_right, ml, mr

    def publish_buttons(self, button):
        btn_soft, btn_scr_up, btn_start, btn_back, btn_scr_down = self.robot.getButtons()

        button_enum = ("Soft_Button", "Up_Button", "Start_Button", "Back_Button", "Down_Button")
        for idx, b in enumerate((btn_soft, btn_scr_up, btn_start, btn_back, btn_scr_down)):
            if b == 1:
                button.value = b
                button.name = button_enum[idx]
                self.buttonPub.publish(button)

    def publish_digital(self, sensor):
        lsb, rsb, lfb, rfb, lw, rw = self.robot.getDigitalSensors()

        sensor_enum = ("Left_Side_Bumper", "Right_Side_Bumper", "Left_Bumper", "Right_Bumper", "Left_Wheel",
        "Right_Wheel")
        for idx, b in enumerate((lsb, rsb, lfb, rfb, lw, rw)):
            if b == 1:
                sensor.value = b
                sensor.name = sensor_enum[idx]
                self.sensorPub.publish(sensor)
        return lw, rw, lsb, rsb, lfb, rfb

    def set_battery_status(self):
        # notify if low batt
        charge = self.robot.getCharger()
        if charge < 10:
            #print "battery low " + str(self.robot.getCharger()) + "%"
            self.robot.setLED("battery", "red", "pulse")
        elif charge < 25:
            self.robot.setLED("battery", "yellow", "solid")
        else:
            self.robot.setLED("battery", "green", "solid")
        rospy.loginfo_throttle(60, "Current battery status: " + str(charge) + "%")

    def violate_safety_constraints(left_drop, right_drop, *digital_sensors ):
        if left_drop > 30 or right_drop > 30:
            print "safety constraint violated by drop sensor"
            return True
        else:
            for sensor in digital_sensors:
                if sensor == 1:
                    print "safety constraint violated by digital sensor"
                    return True
        return False

    def setInfoLed(self, data):
        self.robot.setLED("info", data.color, data.status)
        return SetLedResponse()

    def playSound(self, data):
        self.robot.playSound(data.soundid)
        return PlaySoundResponse()

    def setLDS(self, data):
        if data.data:
            self.robot.setLDS("on")
        else:
            self.robot.setLDS("off")
        return SetBoolResponse(True, "")

if __name__ == "__main__":    
    robot = NeatoNode()
    robot.spin()

