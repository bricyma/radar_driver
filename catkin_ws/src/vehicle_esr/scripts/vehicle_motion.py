#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

pub = rospy.Publisher('/as_rx/vehicle_motion', TwistStamped, queue_size=10)


def callback(data):
    pub.publish(data)


def listener():
    print 'provide can bus information to esr, start!'
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/vehicle/twist', TwistStamped, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
