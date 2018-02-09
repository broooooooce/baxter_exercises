#!/usr/bin/env python

# Collision avoidance suppression function

# Bruce Stracener - University of Arkansas for Medical Sciences
# 01/10/2018


import rospy
from std_msgs.msg import Empty

def ca_suppressor():
    pub1 = rospy.Publisher('robot/limb/right/suppress_collision_avoidance', Empty, queue_size=10)
    pub2 = rospy.Publisher('robot/limb/left/suppress_collision_avoidance', Empty, queue_size=10)
    rospy.init_node('ca_suppressor', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub1.publish()
        pub2.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        ca_suppressor()
    except rospy.ROSInterruptException:
        pass


