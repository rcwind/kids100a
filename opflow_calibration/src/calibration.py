#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import message_filters

class optical_flow_calibration:
    def __init__(self):
        self.optical_flow_sub = message_filters.Subscriber('/optical/data', Float32MultiArray)
        self.odom_sub = message_filters.Subscriber('/odom', Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.optical_flow_sub,self.odom_sub], 10, 0.3, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        self.rate = rospy.Rate(1) # 1Hz 

    def callback(self, opflow_data, odom):
        x1 = opflow_data.data[2]
        x2 = odom.pose.pose.position.x
        y1 = opflow_data.data[3] 
        y2 = odom.pose.pose.position.y
        rospy.loginfo("[opflow, odom] [x: %f, x: %f], [y: %f, y: %f]"%(x1, x2, y1, y2))
        if odom.pose.pose.position.x != 0 and odom.pose.pose.position.y != 0:
            dx = x1 - x2
            dy = y1 - y2

            px = dx / odom.pose.pose.position.x * 100.0
            py = dy / odom.pose.pose.position.y * 100.0

            rospy.logwarn("dx :%f, dy: %f, x error: %f%%, y error: %f%%"%(dx, dy, px, py))

        self.rate.sleep()

def main():
    rospy.init_node('opflow_calibration', anonymous=True)

    ofc = optical_flow_calibration()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

