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
        # this function tries to drive a straight line
        # the distance given is multiplied by 2.5, but the neato should stop at distance
        time.sleep(1)
        self.rel_driven_distance = 0.00
        self.movement.l_dist = 2.5 * distance 
        self.movement.r_dist = 2.5 * distance
        self.movement.vel = 80
        self.cmdDistPub.publish(self.movement)
        # this loop asks if the driven distance measured by the state estimation is larger than the distance 
        # you told the neato to drive 
        while(self.rel_driven_distance < distance):
            pass
        rospy.logwarn("i will now stop")
        self.cmdDistPub.publish(self.stop)


    def turn(self,angle):
        # same as straight() but with an angle
        self.rel_driven_angle = 0.00
        self.movement.vel = 80
        self.movement.l_dist = -20
        self.movement.r_dist = 20
        time.sleep(1)
        self.cmdDistPub.publish(self.movement)
        while(self.rel_driven_angle < angle):
            pass
        rospy.logwarn("i will now stop")
        self.cmdDistPub.publish(self.stop)

    
    def drive_rectangle(self,length):
        for i in range(4):
            self.straight(length)
            self.turn(math.pi/2)

    def odomCb(self,req):
        # this function checks the state estimation for the position of the neato
        # it uses the updates to calculate the driven distance 
        x = req.pose.pose.position.x - self.old_pos_x 
        y = req.pose.pose.position.y - self.old_pos_y
        
        # simply uses the euclidean distance to determine the driven distance
        self.rel_driven_distance += ((x)**2+(y)**2)**0.5

        self.old_pos_x = req.pose.pose.position.x
        self.old_pos_y = req.pose.pose.position.y

        quaternion = (req.pose.pose.orientation.x,
                      req.pose.pose.orientation.y,
                      req.pose.pose.orientation.z,
                      req.pose.pose.orientation.w)# get yaw angle from odom msg
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        rel_turn = (yaw - self.old_yaw)
        self.abs_driven_angle += abs(rel_turn)
        self.rel_driven_angle += abs(rel_turn) 
        self.old_yaw = yaw

if __name__ == "__main__":    
    neato_motion = Motion()
    set_motion = rospy.get_param("~motion")
    time.sleep(3)
    if set_motion == "straight":
        neato_motion.straight(1)
    elif set_motion == "rectangle":
        neato_motion.drive_rectangle(0.3)
    elif set_motion == "turn":
        neato_motion.turn(2*math.pi)
    else:
        rospy.logerr("Invalid arg in motion launch file. Please one of the following arg: 'straight', 'rectangle' or 'turn'.")
