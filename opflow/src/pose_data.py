#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

class optical_flow_conversion:
    def __init__(self):
        self.optical_flow_pub = rospy.Publisher('/optical/pose', Odometry, queue_size=10)
        self.optical_flow_sub = rospy.Subscriber('/optical/data', Float32MultiArray, self.callback)
        self.rate = rospy.Rate(100) # 100Hz 
        self.optical_odom = Odometry()

    def callback(self,data):
        self.current_time = rospy.get_rostime()

        self.optical_odom.header.frame_id = 'optical_link'
        self.optical_odom.header.stamp = self.current_time

        self.optical_odom.pose.pose.position.x = data.data[2]
        self.optical_odom.pose.pose.position.y = data.data[3]
        self.optical_odom.pose.pose.position.z = 0.0
        self.optical_odom.pose.pose.orientation.x = 0.0
        self.optical_odom.pose.pose.orientation.y = 0.0
        self.optical_odom.pose.pose.orientation.z = 0.0
        self.optical_odom.pose.pose.orientation.w = 0.0

        self.optical_odom.pose.covariance[0] = 0.000001
        self.optical_odom.pose.covariance[7] = 0.000001
        self.optical_odom.pose.covariance[14] = 10000000.0
        self.optical_odom.pose.covariance[21] = 10000000.0
        self.optical_odom.pose.covariance[28] = 10000000.0
        self.optical_odom.pose.covariance[35] = 0.03

        self.optical_flow_pub.publish(self.optical_odom)
        self.rate.sleep()

def main():
    rospy.init_node('data_converter_pose', anonymous=True)
    listener = tf.TransformListener()

    ofc = optical_flow_conversion()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
