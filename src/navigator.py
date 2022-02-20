#!/usr/bin/python

import math
import rospy
from control_input_aggregator.msg import ControlInput
from embedded_controller_relay.msg import NavSatReport
from navigation_controller.msg import NavigationCommand
from sensor_msgs.msg import Imu, MagneticField


#test 2

class NavigationController:
    def __init__(self):
        rospy.init_node("navigation_controller")
        rospy.loginfo("Setting up node.")

        #self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.handle_imu_update, queue_size=1)
        self.mag_subscriber = rospy.Subscriber('/imu/mag', MagneticField, self.handle_mag_update, queue_size=1)
        self.gps_subscriber = rospy.Subscriber('/gps', NavSatReport, self.handle_gps_update, queue_size=1)
        self.command_subsciber = rospy.Subscriber('/navigation_command', NavigationCommand, self.handle_nav_command, queue_size=1)

        self.control_input_publisher = rospy.Publisher('/control_input', ControlInput, queue_size=1)

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
        self.target_latitude = 'NaN'
        self.target_longitude = 'NaN'
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

        #print(self.latitude, self.longitude)

    def handle_nav_command(self, data):
        if data.target == 'Gate':
            self.target_type = 'Gate'
        elif data.target == 'Post':
            self.target_type = 'Post'
        else:
            rospy.logwarn("Received invalid navigation command with target type: " + data.target)
            return

        self.target_coordinates = (data.latitude, data.longitude)
        self.target_latitude = data.latitude
        self.target_longitude = data.longitude
        self.target_accuracy = data.accuracy

        print(data)

        self.calculate_target_heading()

    def calculate_target_heading(self):
        # Formula Explanation
        # https://www.movable-type.co.uk/scripts/latlong.html
        # Initial Bearing Formula
        self.heading_goal = ((math.atan2(
            math.sin(self.target_longitude - self.longitude) * math.cos(self.target_longitude),
            math.cos(self.latitude) * math.sin(self.longitude) - math.sin(self.latitude) * math.cos(self.target_longitude) * math.cos(self.target_longitude-self.longitude) 
        ) * 180 / math.pi) + 360) % 360

        print("Headings:", self.heading_goal, self.heading)


    def match_heading(self):
        self.calculate_target_heading()
    
        # Return if rover heading is unknown
        if self.heading is 'NaN':
            return

        # Determine whether to turn left or right
        delta_left = abs(self.heading + 360 - self.heading_goal) if self.heading < self.heading_goal else abs(self.heading - self.heading_goal)
        delta_right = abs(self.heading_goal - self.heading) if self.heading < self.heading_goal else abs(self.heading + 360 - self.heading_goal)

        print("Deltas:", delta_left, delta_right)

        delta = 0

        if delta_left < delta_right:
            delta = -delta_left / 180
        else:
            delta = delta_right / 180

        print(abs(delta))
        if abs(delta) < 20.0/180:
            delta = 0

        control_input = ControlInput()
        control_input.channel = "navigator"
        control_input.heading = (0, delta)
        control_input.speed_clamp = .75
        control_input.is_urgent = False
        
        self.control_input_publisher.publish(control_input)


    def run(self):
        # set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.target_type is not 'NaN':
                self.match_heading()

            rate.sleep()


if __name__ == '__main__':
    controller = NavigationController()
    controller.run()


# rostopic pub /navigation_command navigation_controller/NavigationCommand '{target: "Post", latitude: 0, longitude: 0, accuracy: 0}'