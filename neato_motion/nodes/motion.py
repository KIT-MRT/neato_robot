#!/usr/bin/env python
import rospy
import math
import tf
import time

from geometry_msgs.msg import Quaternion
from neato_node.msg import Movement
from nav_msgs.msg import Odometry

class Motion:
    def __init__(self):
        self.abs_driven_distance = 0.00
        self.rel_driven_distance = 0.00
        self.distance_increment = 0.01
        self.old_pos_x = 0.00
        self.old_pos_y = 0.00
        self.abs_driven_angle = 0.00
        self.rel_driven_angle = 0.00


        self.old_yaw = 0.00
        rospy.init_node('neato_motion')

        self.cmdDistPub = rospy.Publisher('cmd_dist', Movement, queue_size=1)
        self.movement = Movement()

        self.stop = Movement()
        self.stop.l_dist = 0.0
        self.stop.r_dist = 0.0
        self.stop.vel = 0

        rospy.Subscriber("odom", Odometry, self.odomCb)


    def straight(self,distance): 
        time.sleep(10)
        self.rel_driven_distance = 0.00
        self.movement.l_dist = 1.5 * distance 
        self.movement.r_dist = 1.5 * distance
        self.movement.vel = 50
        self.cmdDistPub.publish(self.movement)
        while(self.rel_driven_distance < distance):
            pass
        rospy.logwarn("i will now stop")
        self.cmdDistPub.publish(self.stop)


    def turn(self,angle):
        self.rel_driven_angle = 0.00
        self.movement.l_dist = 1.5*angle 
        self.movement.r_dist = 1.5*angle 
        self.cmdDistPub.publish(self.movement)
        while(self.rel_driven_angle < angle):
            pass
        self.cmdDistPub.publish(self.stop)

    def curve(self,radius):
        circle = Circle(radius)
        lwheeldist,rwheeldist,arc = circle.get_dist()
        while(drivenarc <  arc):
            rospy.publish(motorcommand(lwheeldist,rwheeldist))
    
    def drive_rectangle(self,length):
        for i in range(3):
            self.straight(length)
            self.turn(math.pi/2)

    def odomCb(self,req):
        x = req.pose.pose.position.x - self.old_pos_x 
        y = req.pose.pose.position.y - self.old_pos_y
        
        self.rel_driven_distance = ((x)**2+(y)**2)**0.5

        self.old_pos_x = req.pose.pose.position.x
        self.old_pos_y = req.pose.pose.position.y

        quaternion = (req.pose.pose.orientation.x,
                      req.pose.pose.orientation.y,
                      req.pose.pose.orientation.z,
                      req.pose.pose.orientation.w)# get yaw angle from odom msg
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        rel_turn = (yaw - self.old_yaw)
        self.abs_driven_angle += rel_turn
        self.rel_driven_angle += rel_turn 
        self.old_yaw = yaw

if __name__ == "__main__":    
    neato_motion = Motion()
    neato_motion.drive_rectangle(0.3)
    
        
