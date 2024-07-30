#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import random
import time

points = []
fields = []

def create_point_cloud():
    global points
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"
    return pc2.create_cloud(header, fields, points)

def pointcloud_publisher():
    global fields
    rospy.init_node('pointcloud_publisher', anonymous=True)
    pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)

    for i in range(1000000):
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        z = random.uniform(-10, 10)
        points.append([x, y, z])

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    pc = create_point_cloud()

    while not rospy.is_shutdown():
        pc.header.stamp = rospy.Time.now()
        rospy.loginfo("Publishing PointCloud2 message")
        pub.publish(pc)
        rate.sleep()

if __name__ == '__main__':
    try:
        pointcloud_publisher()
    except rospy.ROSInterruptException:
        pass
