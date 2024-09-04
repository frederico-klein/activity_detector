#!/usr/bin/env python3

import rospy
from insole_msgs.msg import InsoleSensorStamped
rospy.init_node("fake_pressure_publisher")

rate = rospy.Rate(100)

class FakeSidePublisher():
    def __init__(self, side : str):
        self.side = side
        self.pub = rospy.Publisher(self.side+"/insole", InsoleSensorStamped, queue_size=10)

    def publish(self, percentage):
        this_msg = InsoleSensorStamped()
        this_msg.header.stamp = rospy.Time.now()
        this_msg.pressure.data = [0]*16
        if percentage >= 0 and percentage< 20:
            this_msg.pressure.data[0] = 20
            this_msg.pressure.data[1] = 20
        if percentage >= 20 and percentage< 60:
            this_msg.pressure.data[0] = 5
            this_msg.pressure.data[1] = 5
            this_msg.pressure.data[2] = 10
            this_msg.pressure.data[3] = 10
        if percentage >= 60 and percentage< 80:
            for i in range(16):
                this_msg.pressure.data[i] = 2
        if percentage >= 80 and percentage< 100:
            this_msg.pressure.data[13] = 15
            this_msg.pressure.data[14] = 15
            this_msg.pressure.data[15] = 15
        self.pub.publish(this_msg)


i = 0

left = FakeSidePublisher("left")
right = FakeSidePublisher("right")


while not rospy.is_shutdown() :
    left.publish(i%100)
    right.publish((i+50)%100)
    i+=1
    rate.sleep()


