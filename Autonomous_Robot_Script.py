#!/usr/bin/env python

'''
-------------------------------------
Hayden Coe                          |
                                    |
-------------------------------------
'''

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import actionlib 
from kobuki_msgs.msg import BumperEvent
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class turtlebot_object_search:
  
    def __init__(self):
	
	
#------ STEP ONE (DECLARE ITEMS) ------------#
	
	 #------ Windows to display image output
        
      cv2.startWindowThread()  
      
     #------ Current position of the robot
      self.xpos = 0
      self.ypos = 0
      
     #-------- Counter -------
      self.waypointCounter = 0
      

     #---------- Seen flags ------------------------------------------
      self.seen_red = False  
      self.seen_yellow = False
      self.seen_green = False
      self.seen_blue = False 
 
 
      #---------- Found flags ------------------------------------------
      self.found_red = False  
      self.found_yellow = False
      self.found_green = False
      self.found_blue = False
      
      #----------- Other ------------------------------------------
      self.CVbridge = CvBridge()    
   
      self.obstaclePresent = False   
      
      self.twist = Twist()
     
      #------------ Publishers ----------------------------------------    
  
      self.cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
       
      self.moveBasePublish = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)  
        
      #------------ Subscribers ---------------------------------------                                   
       
      #--- Pose Sub    
      self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl)       
       
      #--- RGB Sub
      self.rgb_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                   Image, self.rgbCallback)
      
      #--- Laser Sub                 
      self.las_sub = rospy.Subscriber('/scan', LaserScan, self.las_call)                 
      
      #--- Bump Sub              
      self.bumber_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bump_cb)                 

   
      rospy.sleep(3)# get everything loaded first.
    
    
    '''
     --------------------------------------
    |  Reference for BumperEvent:          |
    |  Documentation for ROS and Turtlebot |
    |  Spring 2016                         |
    |  Para 2.1                            |
     --------------------------------------
    '''  
   
    def bump_cb(self, msg):
       if(msg.state == BumperEvent.PRESSED):
          print("Oh, didn't see that there!")
          move_cmd = Twist()          
          move_cmd.linear.x = 0.7 # set speed
          move_cmd.angular.z = -3.141595 # setting the spin for a 180. 
          self.cmd_vel_pub.publish(self.twist)

   
     #----- Set AMCL data to a variable.
     #---- AMCL tries to match the laser scans to the map which allows for detection if 
     #---- there is any drift occurring in the pose estimate based on the odometry.
     
    def amcl(self, data):
        self.amclData = data   


    #--- Shut down method.
    def shutdown(self):
        print ("Good Bye Friend.")
       
        
    #--- Method for laser calculations.    
    def las_call(self, data):
              
        self.cDis = data.ranges[len(data.ranges) / 2] # in m
        self.laserArray = data.ranges 
        self.leftQuart = len(self.laserArray) / 4
        self.rightQuart = self.leftQuart * 3    
        
    
    
#----- STEP TWO (Colours Defined)-------------------                      
    
   # Method for rospy.spin to callback.           
    def rgbCallback(self, data):
        
        
        self.rgb_data = data
        try:
            #-- get image data from the robot and convert for cv.
            CVimage = self.CVbridge.imgmsg_to_cv2(self.rgb_data, "bgr8")
        except CvBridgeError, e:
            print e
         
                                 
        self.img_to_hsv = cv2.cvtColor(CVimage, cv2.COLOR_BGR2HSV)
     
        #--------- Colour values ----------------------------------------         
        
        #--- values for the colours were obtained by finding the light and dark values 
        #--- of the colours and then experimenting with the vlaues after running in the sim.
                      
        #------ Red values
        self.hsv_lower_red = numpy.array([0,200,100])
        self.hsv_upper_red = numpy.array([5,255,255])
        
        #----- Yellow values
        self.hsv_lower_yellow = numpy.array([30,100,100])
        self.hsv_upper_yellow = numpy.array([30,255,255])
        
        #------ Green values 
        self.hsv_lower_green = numpy.array([45,100,50])
        self.hsv_upper_green = numpy.array([75,255,255])
        
        #------- Blue values
        self.hsv_lower_blue = numpy.array([110,50,50])
        self.hsv_upper_blue = numpy.array([130,255,250])                  
                       
        
    
        #------ Back up of setting the mask to just white.   
        hsv_thresh = cv2.inRange(self.img_to_hsv,
                                     numpy.array((255, 255, 255)),# White
                                     numpy.array((255, 255, 255)))# White

        
        mask = cv2.moments(hsv_thresh)
        
        red = mask
        yellow = mask
        green = mask
        blue = mask
        
        

        #-------- Create thresholded images for each of the colours. 
      
        red_thresh = cv2.inRange(self.img_to_hsv,
                                     self.hsv_lower_red, self.hsv_upper_red)
            
        yellow_thresh = cv2.inRange(self.img_to_hsv,
                                     self.hsv_lower_yellow, self.hsv_upper_yellow)

        blue_thresh = cv2.inRange(self.img_to_hsv,
                                     self.hsv_lower_blue, self.hsv_upper_blue)  
       
        green_thresh = cv2.inRange(self.img_to_hsv,
                                     self.hsv_lower_green, self.hsv_upper_green)
                                     
                                     
        green = cv2.moments(green_thresh)
        blue = cv2.moments(blue_thresh)   
        yellow = cv2.moments(yellow_thresh)       
        red = cv2.moments(red_thresh)
        
        
        '''
         -------------------------------
        |  Reference for centred dot:  |        
        |  www.pyimagesearch.com       |
        |  OpenCV center of contour    |
        |  2016                        |
         -------------------------------
        '''  
        
              
        hsv_thresh, contours, hierachy = cv2.findContours(hsv_thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                               
         #----------- Add a red circle on current target                        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100.0:
                cv2.drawContours(CVimage, contour, -1, (255, 75, 75))# red
       
        
        
        
               
        #--------- If statement for when all the objects have been found so stop.
        if self.found_red == True and self.found_yellow == True and self.found_blue == True and self.found_green == True:
            al = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
            al.cancel_goal()            
            print ("\n\nWhoop, the robot has found all of the different coloured objects.")
            rospy.signal_shutdown("The robot found all of the objects.")
       
       
        '''
         ------------------------------
        |  Reference for colours:      |
        |  From CMP3103 workshop 3     |
         -------------------------------
        '''
        
        #600000 is the area of the object in view.
        #--- Calls set for when coloured objects have reached the set distance.   
        if red['m00'] > 600000 and self.found_red == False: 
            colourWord = "Red"
            self.found_red = self.travelToColour(CVimage, red_thresh, colourWord)
       
        elif blue['m00'] > 600000 and self.found_blue == False:
            colourWord = "Blue"
            self.found_blue = self.travelToColour(CVimage, blue_thresh, colourWord)       
       
       
        elif yellow['m00'] > 600000 and self.found_yellow == False:
            colourWord = "Yellow"
            self.found_yellow = self.travelToColour(CVimage, yellow_thresh, colourWord)
        
        
        elif green['m00'] > 600000 and self.found_green == False:
            colourWord = "Green"
            self.found_green = self.travelToColour(CVimage, green_thresh, colourWord)
        
     
        
        cv2.namedWindow("Robo Cam", 1)      
     
        #------ Show direct image from the robot. 
        cv2.imshow("Robo Cam", CVimage)  
        # wait        
        cv2.waitKey(1)               
        
              
#----- STEP THREE (Travel to Objects Defined)-------------------          
       
       #------- Function to get the robot to travel towards a colour it has found.
    def travelToColour(self, CVimage, hsv_thresh, colourWord):    
        
        self.seenObject(colourWord)
        
        cv2.namedWindow("Thresh View", 2)     
        #------ Show the current colour image thresholded.   
        cv2.imshow("Thresh View", hsv_thresh)
        cv2.waitKey(1)

        # The area of the colours with the hsv mask.
        M = cv2.moments(hsv_thresh)
     
        #----- Set it so that finding the colours takes priority over waypoints
        al = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        al.cancel_goal()
      
        
        #----- Get the image dimensions
        height, width, depth = CVimage.shape
       
       
        #-------- Check that object is present in the thresholded image    
        if M['m00'] < 10000000.0:
            #------ Get the center of the object.
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #----- Create a circle of the center.            
            cv2.circle(CVimage, (cx, cy), 20, (0,0,255), -1)
            cv2.putText(CVimage, "Center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))
            #----- Error calculation (the angle to the center cx).            
            err = cx - width/2
            #----- Twist message to send to the robot telling it to move            
            
            self.twist.linear.x = 0.5 # set speed
            self.twist.angular.z = -float(err) / 100 # setting the turn based on error amount.
            self.cmd_vel_pub.publish(self.twist)
        
            if(self.cDis < 1.50) and (self.cDis >= 0.75):
               print ("Laser Scan: I'm here at " + str(colourWord))        
               return True # Set found flag to true
               
        else:# back up in case the laser scan fails.
            print ("Set Area: I'm here at " + str(colourWord))
            return True # Set found flag to true
            
        return False
        
    #-------- Function for when an object is seen.
    def seenObject(self, colourWord):
                
            
        if colourWord == "Red" and self.seen_red == False:
           self.seen_red = True
           print(str(colourWord) + " has been seen.") # Prints found object.  
        
        elif colourWord == "Yellow" and self.seen_yellow == False:
           self.seen_yellow = True
           print(str(colourWord) + " has been seen.") # Prints found object.  
        elif colourWord == "Green" and self.seen_green == False:
           self.seen_green = True
           print(str(colourWord) + " has been seen.") # Prints found object.  
        elif colourWord == "Blue" and self.seen_blue == False:
           self.seen_blue = True
           print(str(colourWord) + " has been seen.") # Prints found object.  

    
#----- STEP FOUR (Travel to Waypoints Defined)-------------------        
    '''
     --------------------------------------
    |  Reference for waypoint array:       |
    |  Programming Robots with ROS,        |
    |  Chapter 13 On Patrol.               |
     --------------------------------------
    '''
    
    def waypointTravel(self, waypoints):

       
        rospy.on_shutdown(self.shutdown)# ctrl c shutdown.        
        
        go_po = PoseStamped()
        
        # Get the map from the bot
        go_po.header.frame_id = "/map"
       
        
        # Loop through the waypoints
        for goal in waypoints:
            traveling = True
            
            go_po.pose.position.x = goal[0]
            go_po.pose.position.y = goal[1]
            go_po.pose.position.z = 0.0

            print("I'm off to: \n" + str((go_po.pose.position.x, go_po.pose.position.y)))
            
            go_po.pose.orientation = self.amclData.pose.pose.orientation
            self.moveBasePublish.publish(go_po)
           
            
            self.waypointCounter = self.waypointCounter + 1
            print("Number of waypoints visted: " + str(self.waypointCounter))            
            
            while traveling:
              if(-0.5 < (self.amclData.pose.pose.position.x - goal[0]) < 0.5) and (-0.5 < (self.amclData.pose.pose.position.y - goal[1]) < 0.5):
                self.look_left()
                self.look_right()
                traveling = False
            
            
            rospy.sleep(1)
      

    def look_left(self):
          print("Looking Left...")
          turn = 0
          while turn < 8:
             rospy.sleep(0.5)
             spin = Twist()
             spin.angular.z = 1.0
             self.cmd_vel_pub.publish(spin)
             turn = turn+1
          return

    def look_right(self):
          print("Looking Right...")
          turn = 0
          while turn < 8:
             rospy.sleep(0.5)
             spin = Twist()
             spin.angular.z = -1.0
             self.cmd_vel_pub.publish(spin)
             turn = turn+1
          return
          
def start():     
    # Starting code to launch the waypoint method and spin the RGB callback method to run at the same time.
    rospy.init_node("turtlebot_object_search", anonymous=True)
    cv = turtlebot_object_search()
    waypoints = [[2.465, -4.500],[-2.083, -4.500], [1.034, -1.530], [-1.023, 3.563], [1.788, 4.109], [-4.166, 1.083], [2.500, -0.011], [-4.333, 5.020]]
    cv.waypointTravel(waypoints)
    rospy.spin()
    cv2.destroyAllWindows()         
            

if __name__ == '__main__':
   
     start()   
   
   
    
     
    
    
    
    
