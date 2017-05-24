#!/usr/bin/env python
import rospy
import numpy as np
from delphi_srr_msgs.msg import SrrTrack
from radar_msgs.msg import RadarDetectionArray
from radar_msgs.msg import RadarDetection
from geometry_msgs.msg import TwistStamped

pub = rospy.Publisher('/srr_test/vehicle_motion', TwistStamped, queue_size=10)
pub_front_left = rospy.Publisher('/srr_front_left/as_tx/detections', RadarDetectionArray, queue_size=10)
pub_front_right = rospy.Publisher('/srr_front_right/as_tx/detections', RadarDetectionArray, queue_size=10)
pub_rear_left = rospy.Publisher('/srr_rear_left/as_tx/detections', RadarDetectionArray, queue_size=10)
pub_rear_right = rospy.Publisher('/srr_rear_right/as_tx/detections', RadarDetectionArray, queue_size=10)
pub_radar = [pub_front_left, pub_front_right, pub_rear_left, pub_rear_right]

before_nsec = [0]*4
detect_arr = [RadarDetectionArray()]*4


def handler(msg, type):
    cur_nsec = msg.header.stamp.nsecs
    global before_nsec
    global detect_arr

    detect = RadarDetection()
    detect.position.x = msg.CAN_TX_DETECT_RANGE
    detect.position.y = msg.CAN_TX_DETECT_ANGLE
    detect.linear_velocity.x = msg.CAN_TX_DETECT_RANGE_RATE
    detect.amplitude = msg.CAN_TX_DETECT_AMPLITUDE
    detect.linear_velocity.z = msg.CAN_TX_DETECT_VALID_LEVEL
    if cur_nsec != before_nsec[type-1]:
        if len(detect_arr[type-1].detections):
            pub_radar[type-1].publish(detect_arr[type-1])
        detect_arr[type-1] = RadarDetectionArray()
        detect_arr[type-1].header = msg.header
    detect_arr[type-1].detections.append(detect) 
    before_nsec[type-1] = cur_nsec



def callback1(msg):
    handler(msg, 1)

def callback2(msg):
    handler(msg, 2)

def callback3(msg):
    handler(msg, 3)

def callback4(msg):
    handler(msg, 4)


def listener():
    print 'start transformation!'
    rospy.init_node('srr_transform', anonymous=True)
    rospy.Subscriber('/srr_front_left/parsed_tx/srr_track', SrrTrack, callback1)
    rospy.Subscriber('/srr_front_right/parsed_tx/srr_track', SrrTrack, callback2)
    rospy.Subscriber('/srr_rear_left/parsed_tx/srr_track', SrrTrack, callback3)
    rospy.Subscriber('/srr_rear_right/parsed_tx/srr_track', SrrTrack, callback4)
    rospy.spin()


if __name__ == "__main__":
    listener()
