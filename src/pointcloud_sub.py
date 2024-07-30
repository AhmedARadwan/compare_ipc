#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

counter = 0
total_time = 0

def callback(msg):
    global counter
    global total_time
    receive_time = rospy.Time.now()
    sent_time = msg.header.stamp
    latency = (receive_time - sent_time).to_sec()
    # rospy.loginfo(f"Latency: {latency*1000} ms")
    counter += 1
    total_time += latency
    if counter == 1000:
        avg_latency = total_time / 1000
        rospy.loginfo(f"Average Latency: {avg_latency*1000} ms")
        counter = 0
        total_time = 0


def pointcloud_subscriber():
    rospy.init_node('pointcloud_subscriber', anonymous=True)
    rospy.Subscriber('/point_cloud', PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        pointcloud_subscriber()
    except rospy.ROSInterruptException:
        pass
