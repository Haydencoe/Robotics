
# CMP Robotics Assignment 1
# 15595332 Hayden Coe

import rospy,numpy,cv2,time,cv2.cv

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge 

# colours to variable number 
RED = 0
YELLOW = 1
GREEN = 2
BLUE = 3
                         
class Robot:
  
    def __init__(self):
        ############### set way point coordinates to go to specific points 
        self.first_waypoint = [(1.88, -4.53, 0.0),(0.0, 0.0, 1.0, 0.0)],
        self.second_waypoint = [(2.71, 0.48, 0.0),(0.0, 0.0, 1.0, 0.0)],
        self.third_waypoint = [(1.28, 4.45, 0.0), (0.0, 0.0, 1.0 , 0.0)],
        self.forth_waypoint = [(-3.93, 1.46, 0.0), (0.0, 0.0, 1.0 , 0.0)],
        ##################################################################
        self.waypoint1 = False # waypoint flags boolean
        self.waypoint2 = False
        self.waypoint3 = False
        self.waypoint4 = False
        ####################################################################
        self.found_red = False # flag 
        self.found_yellow = False
        self.found_green = False
        self.found_blue = False
        #####################################################################
        self.explores = False # flag 
        self.moving_towards = None
        self.timer = 0 
        self.movingtogoal = True
        self.nav = True # flag 
        self.twist = Twist()
        self.finished = False
        self.defineColVals()
        self.cv_bridge = CvBridge()  
        cv2.namedWindow("Image window", 1) 
        cv2.startWindowThread()
        
         ######################## publishers ############################       
        
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',
                                               PoseStamped, queue_size=1) 
        
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel",
                                           Twist, queue_size=10)
        
        ################### subscribers ##############################                                   
       
       self.rgb_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",
                                   Image, self.rgbCallback)
        
        self.laser_sub = rospy.Subscriber("/turtlebot/scan",
                                          LaserScan, self.laserCallback)
       
       #################### define colour values ##################
    
    def defineColVals(self):
        
        # Red values
        self.hsv_lower_red = numpy.array([0,100,100])
        self.hsv_upper_red = numpy.array([5,255,140])
        
        # Yellow values
        self.hsv_lower_yellow = numpy.array([30,100,100])
        self.hsv_upper_yellow = numpy.array([30,255,255])
        
        # Green values 
        self.hsv_lower_green = numpy.array([50,100,50])
        self.hsv_upper_green = numpy.array([70,255,255])
        
        # Blue alues
        self.hsv_lower_blue = numpy.array([110,50,50])
        self.hsv_upper_blue = numpy.array([130,255,250])
      
      ######################## define the laser use ###############                                            
    
    def laserCallback(self, data):
        self.laser_data = data
        
    def rgbCallback(self,data):
        self.rgb_data = data
        if self.finished:
            self.depth_sub.unregister()
            self.rgb_sub.unregister()
            self.cmd_vel_pub.unregister()
            print("Finished.")
            return
            
        self.image = self.cv_bridge.imgmsg_to_cv2(self.rgb_data,desired_encoding="bgr8")
        self.image_to_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # Build HSV masks from range of colour values
        red_hsv_mask = cv2.inRange(self.image_to_hsv,
                                   self.hsv_lower_red, self.hsv_upper_red)
        yellow_hsv_mask = cv2.inRange(self.image_to_hsv, 
                                      self.hsv_lower_yellow, self.hsv_upper_yellow)
        green_hsv_mask = cv2.inRange(self.image_to_hsv, 
                                     self.hsv_lower_green, self.hsv_upper_green)
        blue_hsv_mask = cv2.inRange(self.image_to_hsv,
                                    self.hsv_lower_blue, self.hsv_upper_blue)
        
        self.h, self.w, d = self.image.shape
       
       # Calculate object centre to ensure red dot location finder is in the centre. 
        moments_list = {RED: 0.0, YELLOW: 0.0, GREEN: 0.0, BLUE: 0.0}
        
       #  Way point 1 needs a count of approx (30)
        if self.waypoint1 == False and self.nav == True: 
            print("We are off to waypoint 1")
            for x in range(30): # counts for 30 steps 
                for pose in self.first_waypoint:
                    goal = self.goal_pose(pose)
                    self.goal_pub.publish(goal)
                    self.waypoint1 = True 
                    self.explores = True
                    self.nav = False
                       
        if self.waypoint2 == False and self.nav == True: 
            print("We are off to waypoint 2")
            for x in range(25): # count for 25 steps 
                for pose in self.second_waypoint:
                    goal = self.goal_pose(pose)
                    self.goal_pub.publish(goal)
                    self.waypoint2 = True 
                    self.explores = True
                    self.nav = False
                    
        if self.waypoint3 == False and self.nav == True: 
            print("We are off to waypoint 3")
            for x in range(30):
                for pose in self.third_waypoint:
                    goal = self.goal_pose(pose)
                    self.goal_pub.publish(goal)
                    self.waypoint3 = True 
                    self.explores = True
                    self.nav = False  
                    
        if self.waypoint4 == False and self.nav == True: 
            print("We are off to waypoint 4")
            for x in range(50):
                for pose in self.forth_waypoint:
                    goal = self.goal_pose(pose)
                    self.goal_pub.publish(goal)
                    self.waypoint4 = True 
                    self.explores = True
                    self.nav = False           
          
          ################## Waypoint actions finished ##################          
        
        if not(self.found_red): #### calculates range to object centre ######
            red_M = cv2.moments(red_hsv_mask)
            if red_M['m00'] > 0:
                moments_list[RED] = red_M['m00']
                
        else:
            red_M = {'m00', 0.0}
        if not(self.found_green):
            green_M = cv2.moments(green_hsv_mask)
            if green_M['m00'] > 0:
                moments_list[GREEN] = green_M['m00']
        else:
            green_M = {'m00', 0.0}
        if not(self.found_yellow):
            yellow_M = cv2.moments(yellow_hsv_mask)
            if yellow_M['m00'] > 0:
                moments_list[YELLOW] = yellow_M['m00']
        else:
            yellow_M = {'m00', 0.0}
        if not(self.found_blue):
            blue_M = cv2.moments(blue_hsv_mask)
            if blue_M['m00'] > 0:
                moments_list[BLUE] = blue_M['m00']
        else:
            blue_M = {'m00', 0.0}
           
        if (sum(moments_list) > 0.0):
            
            if ((not(self.found_red) or (self.moving_towards == RED)) and
                (red_M['m00'] > 50000.0)):
                self.moveTowards(red_M, RED)
                
            elif ((not(self.found_green) or (self.moving_towards == GREEN)) and
                (green_M['m00'] > 50000.0)):
                self.moveTowards(green_M, GREEN)
                
            elif ((not(self.found_yellow) or (self.moving_towards == YELLOW)) and
                (yellow_M['m00'] > 50000.0)):
                self.moveTowards(yellow_M, YELLOW)
               
            elif ((not(self.found_blue) or (self.moving_towards == BLUE)) and
                (blue_M['m00'] > 50000.0)):
                self.moveTowards(blue_M, BLUE)
                
            self.cmd_vel_pub.publish(self.twist)
            cv2.imshow("Image window", self.image)
      
        cv2.waitKey(1)
            
    def explore(self):

       ##### publishes a twist command 
        self.cmd_vel_pub.publish(self.twist)
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        time.sleep(1)
        self.cmd_vel_pub.publish(self.twist)
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        time.sleep(1)
        
        try:
            
            ranges = self.laser.data           #get the range data from the scanner
            leftRanges = ranges[center-10:center]    #get the ranges for the right side of the image
            rightRanges = ranges[center:center+10]     #get the ranges for the left side of the image
            middle = int(round((len(ranges)-1)/4))
            if numpy.nanmin(leftRanges) > 2.0 and numpy.nanmin(rightRanges) > 2.0: # if the area in front is fine
                print "explore forward"
                self.twist.linear.x = 0.5
                self.twist.angular.z = 0.0
#             Turns the robot left when something is on the right of the robot.
            elif numpy.nanmin(leftRanges) > 2.0 and numpy.nanmin(rightRanges) < 2.0: # if the area in front isnt ok 
                print "explore left"
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.5
            # Turns the robot right when something is on the left.
            elif numpy.nanmin(leftRanges) < 2.0 and numpy.nanmin(rightRanges) > 2.0: # 
                print "explore right"
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.5
            # If the robot is unsure about ranges will spin until sure.
            else:
                    print "explore recover"
                    self.twist.linear.x = 0.0
                              
        except NameError:
            print("Here we go")
            
    def moveTowards(self, M, colour):
        
        if not (self.moving_towards == colour):
                #print("at least one of the colours was found...")
                self.cmd_vel_pub.publish(self.twist)
                self.twist.angular.z = 0.0
                self.twist.linear.x = 0.0
        print(str(colour) + " is seen: " ) ### prints to screen### 
        if M['m00'] < 10000000.0:
            #print(str(colour) + " < 2500000.0")
            self.moving_towards = colour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(self.image, (cx,cy), 20, (0,0,255), -1)
            err = cx - self.w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 50
             
        else:
            self.setFound(colour)
                       
    def setFound(self, colour):
        if colour == 0: # simple if statements defining what happens when a certain colour is reached 
            self.found_red = True # if found_red set value to true flag 
            print "Cool ive found Red" # print to screen statement 
            self.twist.linear.x = 0.0 # found colour sends twist motion 
            #self.twist.angular.z = 0.0
            self.moving_towards = None # if moving_towards flag set
            if self.moving_towards == None: # if flag is set to None two other flags are set 
                self.nav = True
                self.explores = False
                
        elif colour == 1:
            self.found_yellow = True
            print "Cool ive found Yellow"
            self.twist.linear.x = 0.0
            #self.twist.angular.z = 0.0
            self.moving_towards = None 
            if self.moving_towards == None:
                self.nav = True
                self.explores = False
#               
        elif colour == 2: 
            self.found_green = True
            print "Cool ive found Green"
            self.twist.linear.x = 0.0
            #self.twist.angular.z = 0.0
            self.moving_towards = None 
            if self.moving_towards == None:
                self.nav = True
                self.explores = False
#           
        
        elif colour == 3:
            self.found_blue = True
            print "Cool ive found Blue"
            self.twist.linear.x = 0.0
           # self.twist.angular.z = 0.0
            self.moving_towards = None 
            if self.moving_towards == None:
                self.nav = True
                self.explores = False
#            
    def goal_pose(self, pose):
            next_goal = PoseStamped()
            next_goal.header.frame_id = '/map'
            next_goal.pose.position.x = pose[0][0]
            next_goal.pose.position.y = pose[0][1]
            next_goal.pose.position.z = pose[0][2]
            next_goal.pose.orientation.x = pose[1][0]
            next_goal.pose.orientation.y = pose[1][1]
            next_goal.pose.orientation.z = pose[1][2]
            next_goal.pose.orientation.w = pose[1][3]    
            return next_goal            
       
rospy.init_node("rgb_converter")
Robot()
rospy.spin()
cv2.destroyAllWindows()
