#!/usr/bin/python

import math
import rospy
from sensor_msgs.msg import Imu, MagneticField


class NavigationController:
    def __init__(self):
        rospy.init_node("navigation_controller")
        rospy.loginfo("Setting up node.")

        #self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.handle_imu_update, queue_size=1)
        self.magn_subscriber = rospy.Subscriber('/imu/mag', MagneticField, self.handle_mag_update, queue_size=1)
    
    def handle_imu_update(self, data):
        # Imu MSG Contents
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
        print(data.orientation)
        print("----------------------------")

    def handle_mag_update(self, data):
        # MagneticFiled MSG Contents
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html
        x = data.magnetic_field.x
        y = data.magnetic_field.y
        z = data.magnetic_field.z

        heading = 'NaN'

        if y > 0:
            heading = 90 - math.atan(x/y) * 180 / math.pi
        elif y < 0:
            heading = 270 - math.atan(x/y) * 180 / math.pi
        else:
            if x < 0:
                heading = 180
            elif x >= 0:
                heading = 0
        
        print(heading)

    def run(self):
        # set the control rate
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    controller = NavigationController()
    controller.run()