#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class optical_flow_conversion:
    def __init__(self):
        self.optical_flow_pub = rospy.Publisher('/optical/odom', Odometry, queue_size=10)
        self.optical_flow_sub = rospy.Subscriber('/optical/data', Float32MultiArray, self.callback)
        self.rate = rospy.Rate(100) # 100Hz 
        self.optical_odom = Odometry()

    def callback(self,data):
        self.current_time = rospy.get_rostime()

        self.optical_odom.header.frame_id = 'optical'
        self.optical_odom.header.stamp = self.current_time

        self.optical_odom.pose.pose.position.x = data.data[2]
        self.optical_odom.pose.pose.position.y = data.data[3]
        self.optical_odom.pose.pose.position.z = 0.0
        self.optical_odom.pose.pose.orientation.x = 0.0
        self.optical_odom.pose.pose.orientation.y = 0.0
        self.optical_odom.pose.pose.orientation.z = 0.0
        self.optical_odom.pose.pose.orientation.w = 0.0

        self.optical_odom.pose.covariance[0] = 0.001
        self.optical_odom.pose.covariance[7] = 0.001
        self.optical_odom.pose.covariance[14] = 1000000.0
        self.optical_odom.pose.covariance[21] = 1000000.0
        self.optical_odom.pose.covariance[28] = 1000000.0
        self.optical_odom.pose.covariance[35] = 0.03

        self.optical_odom.twist.twist.linear.x = data.data[0]
        self.optical_odom.twist.twist.linear.y = data.data[1]
        self.optical_odom.twist.twist.linear.z = 0.0
        self.optical_odom.twist.twist.angular.x = 0.0
        self.optical_odom.twist.twist.angular.y = 0.0
        self.optical_odom.twist.twist.angular.z = 0.0

        self.optical_odom.twist.covariance[0] = 0.001
        self.optical_odom.twist.covariance[7] = 0.001
        self.optical_odom.twist.covariance[14] = 0.001
        self.optical_odom.twist.covariance[21] = 1000000.0
        self.optical_odom.twist.covariance[28] = 1000000.0
        self.optical_odom.twist.covariance[35] = 0.03

        # self.optical_odom.twist.covariance[0] = 0.001
        # self.optical_odom.twist.covariance[7] = 0.001
        # self.optical_odom.twist.covariance[14] = 0.001
        # self.optical_odom.twist.covariance[21] = 1000000.0
        # self.optical_odom.twist.covariance[28] = 1000000.0
        # self.optical_odom.twist.covariance[35] = 0.03

        self.optical_flow_pub.publish(self.optical_odom)
        self.rate.sleep()

def main():
    rospy.init_node('data_converter', anonymous=True)
    listener = tf.TransformListener()

    ofc = optical_flow_conversion()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

