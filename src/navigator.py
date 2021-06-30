#!/usr/bin/python

import math
import rospy
from embedded_controller_relay.msg import NavSatReport
from navigation_controller.msg import NavigationCommand
from sensor_msgs.msg import Imu, MagneticField


class NavigationController:
    def __init__(self):
        rospy.init_node("navigation_controller")
        rospy.loginfo("Setting up node.")

        #self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.handle_imu_update, queue_size=1)
        self.mag_subscriber = rospy.Subscriber('/imu/mag', MagneticField, self.handle_mag_update, queue_size=1)
        self.gps_subscriber = rospy.Subscriber('/gps_test', NavSatReport, self.handle_gps_update, queue_size=1)
        self.command_subsciber = rospy.Subscriber('/navigation_command', NavigationCommand, self.handle_nav_command, queue_size=1)

        #-----------------------
        # Rover Data Storage
        #-----------------------
        # Magnetometer Data
        self.heading = 'NaN'

        # GPS Data
        self.latitude = 'NaN'
        self.longitude = 'NaN'

        # Target Navigation Information
        self.target_type = 'NaN'
        self.target_coordinates = 'NaN'
        self.target_accuracy = 'NaN'

        # Heading Follow Information
        self.heading_goal = 'NaN'


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

        self.heading = 'NaN'

        if y > 0:
            self.heading = 90 - math.atan(x/y) * 180 / math.pi
        elif y < 0:
            self.heading = 270 - math.atan(x/y) * 180 / math.pi
        else:
            if x < 0:
                self.heading = 180
            elif x >= 0:
                self.heading = 0
        
        #print(self.heading)

    def handle_gps_update(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

        print(self.latitude, self.longitude)

    def handle_nav_command(self, data):
        if data.target == 'Gate':
            self.target_type = 'Gate'
        elif data.target == 'Post':
            self.target_type = 'Post'
        else:
            rospy.logwarn("Received invalid navigation command with target type: " + data.target)
            return

        self.target_coordinates = (data.latitude, data.longitude)
        self.target_accuracy = data.accuracy

        print(data)

    def run(self):
        # set the control rate
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    controller = NavigationController()
    controller.run()