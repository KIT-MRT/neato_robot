import rospy
import math
from ..shapes.shapes import Rectangle, Circle
from geometry_msgs.msg import Quaternion
from neato_node.msg import Movement
from nav_msgs.msg import Odometry

class Motion:
    def __init__(self):
        self.driven_distance = 0
        self.distance_increment = 0.01
        self.old_pos_x = 0.00
        self.driven_angle = 0.00
        self.old_yaw = 0.00
        rospy.init_node('neato_motion')


        self.cmdDistPub = rospy.Publisher('cmd_dist', Movement, queue_size=1)
        self.movement = Movement()

        rospy.Subscriber("odom",Odometry,self.odomCb)


    def straight(self,distance):
        self.driven_distance = 0.00
        self.movement.l_dist = self.distance_increment
        self.movement.r_dist = self.distance_increment
        self.movement.vel = 50
        while(self.driven_distance < distance):
            self.cmdDistPub.publish(self.movement)
            


    def turn(self,angle):
        self.driven_angle = 0.00
        self.movement.l_dist = -self.distance_increment
        self.movement.r_dist = self.distance_increment
        self.movement.vel = 50
        while(self.driven_angle < angle):
            self.cmdDistPub.publish(self.movement)

    def curve(self,radius):
        circle = Circle(radius)
        lwheeldist,rwheeldist,arc = circle.get_dist()
        while(drivenarc <  arc):
            rospy.publish(motorcommand(lwheeldist,rwheeldist))
    
    def drive_rectangle(self,length):
        for i in range(3):
            self.turn(math.pi/2)
            self.straight(length)

    def odomCb(self,req):
        # get x position from odom msg
        self.driven_distance += (req.pose.pose.position.x - self.old_pos_x)
        self.old_pos_x = req.pose.pose.position.x

        # get yaw angle from odom msg
        quaternion = Quaternion()
        req.pose.pose.orientation = quaternion
         (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        self.driven_angle += (yaw - self.old_yaw)
        self.old_yaw = yaw

if __name__ == "__main__":    
    neato_motion = Motion()
    neato_motion.straight(1)
    
        
