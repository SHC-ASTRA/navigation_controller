#!/usr/bin/python
import math
import rospy
import time
from control_input_aggregator.msg import ControlInput
from embedded_controller_relay.msg import NavSatReport
from embedded_controller_relay.srv import SignalColor, SignalColorResponse
from geopy import distance
from navigation_controller.msg import NavigationCommand
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool, String, int16 #NEW LIBRARY BUT SHOULD WORK 
from auton.msg import aruco #check this
### The rover knows where it is at all times. 
# It knows this because it knows where it isn't. 
# By subtracting where it is from where it isn't, or where it isn't from where it is (whichever is greater), it obtains a difference, or deviation. 
# The guidance subsystem uses deviations to generate corrective commands to drive the rover from a position where it is to a position where it isn't, 
# and arriving at a position where it wasn't, it now is. 
# Consequently, the position where it is, is now the position that it wasn't, 
# and it follows that the position that it was, is now the position that it isn't.
#In the event that the position that it is in is not the position that it wasn't, 
# the system has acquired a variation, the variation being the difference between where the rover is, and where it wasn't. 
# If variation is considered to be a significant factor, it too may be corrected by the GEA. However, the rover must also know where it was.
#The rover guidance computer scenario works as follows. 
# Because a variation has modified some of the information the rover has obtained, it is not sure just where it is. 
# However, it is sure where it isn't, within reason, and it knows where it was. 
# It now subtracts where it should be from where it wasn't, or vice-versa, 
# and by differentiating this from the algebraic sum of where it shouldn't be, 
### and where it was, it is able to obtain the deviation and its variation, which is called error.



vectorize = lambda a,b : [a[0]-b[0],a[1]-b[1]] #Makes a vector of the difference between spots dlat and dlng

class Navigation_handler:
    ###A class that will handle rover autonomous controls 
    #Strategy: upon recieving the command to navigate as well as target coordinates, triangulates heading and necessary adjustment
    #Upon turning to face the goal, moves forward and recalculates for any new error.
    ###
    def __init__(self):
        #Setup ROS node 
        rospy.init_node("navigation_controller")
        rospy.loginfo("Setting up node.")


        #Setup ros subscribers and publisher 
        #self.mag_subscriber = rospy.Subscriber('/sensor/zed2/zed_node/imu/mag', MagneticField, self.handle_mag_update, queue_size=1)
        self.gps_subscriber = rospy.Subscriber('/teensy/gps', NavSatReport, self.handle_gps_update, queue_size=1)
        self.command_subsciber = rospy.Subscriber('/navigation_command', NavigationCommand, self.handle_nav_command, queue_size=1)
        self.control_input_publisher = rospy.Publisher('/autonomous_control', ControlInput, queue_size=1)
        self.signal_service = rospy.ServiceProxy('/signal_operating_mode', SignalColor)
        self.isAruco = rospy.Subscriber('/arucoToggle',Bool,self.toggleAruco)#NEW
        self.goal_Reached_Publisher = rospy.Publisher('/goal_reached',String)#NEW
        self.aruco_tolerance_sub = rospy.Subscriber('/nav_tol',int16,self.handleArucoTolUpdate)#NEW
        #ADD LISTENER FOR ARUCO HERE
        self.aruco_listener_sub = rospy.Subscriber('/aruco_detector',aruco,self.updateAruco) #Check this
        # Control States
        self.forward_err = 0.9 #Scale multiple to limit how far we go to allow error on angle
        self.control_state = 'Idle' 
        self.target_type = 'NaN'
        self.Currcoords = [] #Current Coordinates
        self.targetLoc = [] #Target coordinates 
        self.targetAcc = 1.5 #Meters allowed
        self.prevCoords = [] #Previous coordinates
        self.err_ang = 0.0 #Angle of error from the current heading and the target heading
        self.heading =[] #Azimuth of the rover
        self.canMove = True #Handles wether or not we can allow rover to update the gps (wait for it to turn before continuing)
        self.canArucoNav = False #For if the waypoint is GNSS only or aruco search after arrival
        self.Aruco_found = False #If the aruco tag is detected
        self.aruco_tolerance = 5 #Max tolerance on how far to search around the rover 
        self.ring = 0 #tells what ring of the spiral the rover is on
        self.aruco_radius = 0.5*5    #Set this to 0.5*x distance at half of max z
        self.aruco_dist = []
    
    def updateAruco(self,data):     
        if data.id <= 6:
            self.Aruco_found = True
            self.aruco_dist = [data.x,data.y,data.z]

    def handleArucoTolUpdate(self,data):
        self.aruco_tolerance = data.data
    def handle_gps_update(self,data):
        #callback for when we get new gps data
        if self.canMove:
            if self.Currcoords: 
                self.prevCoords = self.Currcoords
            
            lat = float(data.latitude)
            lng = float(data.longitude)
            self.Currcoords =[lat,lng]

            print(self.Currcoords)    
        


    def handle_nav_command(self,data):
        #callback for new navigation command sent 
        if data.target != 'Abort' and data.target!='NaN':
            self.target_type=data.target
            lat= float(data.latitude)
            lng = float(data.longitude)
            self.targetLoc = [lat,lng]
            self.targetAcc = data.accuracy
            self.signal_service("Autonomous")
            self.control_state = "Navigate"
        elif data.target =='Abort':
            self.target_type='NaN'
            self.control_state = 'Idle'
            self.signal_service("Idle")
        else:
            rospy.logwarn("Invalid navigation command")
        return

    def toggleAruco(self,data):
            self.canArucoNav = data.data
    def handle_goal_reached(self):
        self.goal_Reached_Publisher.publish("Goal Reached!")
    
    def getGoalDist(self,coord1,coord2):
        return float(distance.distance((coord1[0],coord1[1]),(coord2[0],coord2[1])).m) 

    

    def adjust_direction(self):
        ### A secondary triangulation method to find heading and error
        #Finds the heading by placing the current and previous rover coordinates into a vector
        #Takes the dot product of the vector from current to target and the heading
        #Then takes the cross product of the same
        #Then finds the error angle by taking the arc tangent of the cross and dot prodcts
        ###
        self.canMove = False
        self.heading = vectorize(self.Currcoords,self.prevCoords)
        target_vector = vectorize(self.targetLoc,self.Currcoords)
        #dot = self.heading[0] *(self.targetLoc[0]-self.Currcoords[0])+self.heading[1]*(self.targetLoc[1]-self.Currcoords[1])
        #cross = self.heading[0] *(self.targetLoc[0]-self.Currcoords[0])-self.heading[1]*(self.targetLoc[1]-self.Currcoords[1])
        dot  = self.heading[0] * target_vector[0] + self.heading[1] * target_vector[1] #calculate based off of heading
        cross = self.heading[0] * target_vector[1] - self.heading[1] * target_vector[0] 
        self.err_ang = math.degrees(math.atan2(cross,dot))
        print("ANGLE OF ERROR\t" + str(self.err_ang))
        self.publishController()
        if self.getGoalDist(self.Currcoords,self.targetLoc) > self.getGoalDist(self.prevCoords,self.targetLoc):
            self.err_ang = 180
            self.publishController()
        self.canMove =True
        
        
        
    def publishController(self,speed=0.35, heading=0):
        ###
        #Tells the rover to move
        ##
        if heading==0:
            heading=(self.getGoalDist(self.Currcoords,self.targetLoc),self.err_ang) if self.err_ang != 0 else (self.getGoalDist(self.Currcoords,self.targetLoc)*self.forward_err,self.err_ang)
        control_input = ControlInput()
        control_input.channel='navigator'
        control_input.heading = heading
        control_input.speed_clamp = speed
        control_input.is_urgent = False
        self.control_input_publisher.publish(control_input)

    def run_navigation(self):
        ###
        #Handles the control loop for navigation 
        ###
        rate = rospy.Rate(3) #Place holder for now 
        
        gpsTargetFound = False
        while not rospy.is_shutdown():  
            if self.control_state=='Navigate': #If we are navigating
                if not gpsTargetFound:
                    self.err_ang = 0 #Set error to zero since we are naivelly going forward at first
                    self.publishController() #Go forward (hopefully)
                    print("DISTANCE FROM TARGET" + str(self.getGoalDist(self.Currcoords,self.targetLoc)))
                    self.adjust_direction() #Turn to face the target, strat
                if self.getGoalDist(self.Currcoords,self.targetLoc) < self.targetAcc and self.target_type=='Post': #Check if we have reached the goal
                    if not self.canArucoNav:
                        self.control_state="Goal"
                    else:
                        gpsTargetFound = True
                        self.spiral_nav()
                            
            elif self.control_state=="Goal":
                self.signal_service("goal") #Check if this is the correct syntax
                self.handle_goal_reached()
            rate.sleep()
    
    def spiral_nav(self):
        ### Searches for the aruco tag after making it to the gps waypoint
        #Moves in square spiral within the aruco tolerance 
        # if end of tolerance reached, resets and starts again
        #If aruco tag is found, move towards it 
        ###
        
        self.canMove=False
        if self.Aruco_found:
            goal_heading = math.acos(self.aruco_dist[0]/self.aruco_dist[2])
            self.publishController(heading=goal_heading)
            if self.target_type == 'Post':
                if self.aruco_dist[2] < 3:
                    self.control_state='Goal'
                self.control_state=='Goal'
            else:
                self.handle_Gate()
        else:
            if self.ring==0:
                self.err_ang=360
                self.publishController(speed=0.1)#See if this sends fwd
                self.ring+=1
            else:
                ring_dist = self.ring*self.aruco_radius
                self.publishController(heading=ring_dist)
                for i in range(0,3):
                    self.err_ang = 90
                    self.publishController()
                    time.sleep(0.25)
                    self.err_ang =0
                    if i ==0:
                        if self.ring==1:
                            ring_dist =ring_dist/2
                        else:
                            ring_dist = (self.ring+1)*(self.aruco_radius)
                    elif i==1:
                        ring_dist = ring_dist*2
                    self.publishController(heading=ring_dist)
                    time.sleep(0.5)
                self.ring+=1


        self.canMove=True
    def handle_Gate(self):
        self.canMove=False
        self.err_ang = 45
        self.publishController()
        self.err_ang =0
        time.sleep(0.5)
        for i in range(0,4):
            self.publishController()
        time.sleep(0.5)
        self.err_ang = 90
        self.publishController()
        time.sleep(0.5)
        self.err_ang = 0
        for i in range(0,10):
            self.publishController()
        self.control_state="Goal"
        self.canMove = True





navigator = Navigation_handler()
navigator.run_navigation()