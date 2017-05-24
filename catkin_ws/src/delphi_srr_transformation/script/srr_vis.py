#!/usr/bin/env python
import rospy
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from radar_msgs.msg import RadarDetectionArray


class SrrVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.pub_front_left = rospy.Publisher('/srr_front_left/detection', MarkerArray, queue_size=1)
        self.pub_front_right = rospy.Publisher('/srr_front_right/detection', MarkerArray, queue_size=1)
        self.pub_rear_left = rospy.Publisher('/srr_rear_left/detection', MarkerArray, queue_size=1)
        self.pub_rear_right = rospy.Publisher('/srr_rear_right/detection', MarkerArray, queue_size=1)
        rospy.Subscriber('/srr_front_left/as_tx/detections', RadarDetectionArray, self.callback1)
        rospy.Subscriber('/srr_front_right/as_tx/detections', RadarDetectionArray, self.callback2)
        rospy.Subscriber('/srr_rear_left/as_tx/detections', RadarDetectionArray, self.callback3)
        rospy.Subscriber('/srr_rear_right/as_tx/detections', RadarDetectionArray, self.callback4)

        self.loop = [0] * 4
        self.translation = [(3.66, 1.02, 0), (3.66, -1.02, 0), (-0.94, 1.05, 0), (-0.94, -1.05, 0)]
        self.rotation = [53.4, 100, 100, 323.7]

    def handler(self, msg, id):
        detections = msg.detections
        rate = 1.0
        marker_array = MarkerArray()
        count = 0
        for detect in detections:
            count += 1
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "srr"
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.2)

            marker.id = self.loop[id] * 100 + count
            marker.type = Marker.SPHERE

            detect_range = detect.position.x
            angle = detect.position.y / 180 * math.pi
            lx = detect_range * math.sin(angle) * rate
            ly = detect_range * math.cos(angle)

            marker.pose.position.x = lx * math.cos(self.rotation[id]) + ly * math.sin(self.rotation[id]) + self.translation[id][0]
            marker.pose.position.y = -lx * math.sin(self.rotation[id]) + ly * math.cos(self.rotation[id]) + self.translation[id][1]
            marker.pose.position.z = 0

            delta = 0.02
            marker.scale.x = 0.1 + delta
            marker.scale.y = 0.1 + delta
            marker.scale.z = 1

            marker.color.a = 1.0
            marker.color.r = 250.0/255
            marker.color.g = 210.0/255
            marker.color.b = 3.0/255
            marker_array.markers.append(marker)
        self.pub_object.publish(marker_array)

    def callback1(self, msg):
        self.loop[0] += 1
        hasattr(msg, 0)

    def callback2(self, msg):
        self.loop[0] += 1
        hasattr(msg, 1)

    def callback3(self, msg):
        self.loop[0] += 1
        hasattr(msg, 2)

    def callback4(self, msg):
        self.loop[0] += 1
        hasattr(msg, 3)

if __name__ == '__main__':
    rospy.init_node("srr_visualization")
    _ = SrrVisualizer()
    rospy.spin()
