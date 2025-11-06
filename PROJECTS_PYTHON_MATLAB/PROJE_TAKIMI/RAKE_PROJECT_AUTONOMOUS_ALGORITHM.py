#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool, Float32
from rake_gpio.msg import Detection 
from geometry_msgs.msg import Twist
from rake_ai.msg import BoolStamped



class MovementControl():
    def __init__(self):
        rospy.init_node('rake_TIKA_movement', anonymous=True)
        self.length = 10                            # meter (user defines)
        self.length_among_rows = 5                  # the length of among rows (user defines)

        self.turn = 0 
        self.batteryLevel = 0

    #    rospy.Rate(1)

        # Defined serial properties and created socket object.
   
        self.vel = Twist()
        print("checkpoint1")

        # linear.x and angular.z should be [-1, 1]

        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0

        #Default: No detection, Front Cam Active, Information of servo turn is received and it is okay.

        
        self.servo_engine = True
        self.is_there_path = True
        self.activation_data = BoolStamped() 
        #No movement, yet.
        self.wentRoad=0
        self.totalDistance = 0
        self.random = 0

        self.k = 0
        self.l = 0
        
        #Angle data from servo, back and front cam activation data, and motor velocity commands are published.

        self.publisherCmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.publisherCamera = rospy.Publisher("/activation_data",BoolStamped, queue_size=1)
        self.distance_publisher = rospy.Publisher("/total_distance", Float32, queue_size=10)
        self.battery = rospy.Publisher("/battery_level", Float32, queue_size=10)
        #Battery capacity is calculated for approximately 45 min (changeable)
        
        #Important for calculate wentRoad.
        self.initial_t = rospy.get_time()


        #Data which is about whether there is a front/back path or not and 
        #data which is about turn of servo is okay, are received.
        self.path = rospy.Subscriber("/is_there_detection", Detection, self.callback_path)
        print("checkpoint2")
        
        
    #Callback Functions for Data that is required to be subscribed.
    def callback_path(self,data):
        self.is_there_path= data.detectionData

    def Turn(self):
        #Right Turn.

        self.vel.linear.x = 0
        if self.turn%2==0:
            print("checkpoint6")
            self.vel.angular.z = 1
            self.vel.linear.x = 1
            self.activation_data.data = False
            print("checkpoint7")
            if self.is_there_path == True:
                #A path detected from back cam.
                print("checkpoint10")
                self.vel.linear.x = 0 #Eger kayma olursa buraya sleep atmamız gerekebilir 
                self.vel.angular.z = 0

        #Left Turn.            
        else:
            print("checkpoint5")
            self.vel.angular.z = -1 
            self.vel.linear.x = 1
            self.activation_data.data = True
            print("checkpoint12")
            if self.is_there_path == True:
                print("checkpoint11")
                self.vel.linear.x = 0 #Eger kayma olursa buraya sleep atmamız gerekebilir 
                self.vel.angular.z = 0
                        

    def Linear(self):    
        if self.length > self.wentRoad:
            self.time_control = rospy.get_time()
            self.vel.linear.x = 1
            self.wentRoad = (self.time_control-self.initial_t)*self.vel.linear.x            

        else:
            self.initial_t = rospy.get_time()
            print("path completed")
            self.Turn()      
            print("out of turn {}".format(self.turn))
            self.wentRoad=0
            self.k += 1
            self.l += 1                   
            self.turn += 1
            self.initial_t = rospy.get_time()
          


if __name__ == '__main__': 
    rake = MovementControl()
    rake.vel.linear.x = 0
    rake.vel.angular.z = 0
    while not rospy.is_shutdown():
        
        rake.publisherCmdVel.publish(rake.vel)
        rospy.sleep(0.1)
        rake.distance_publisher.publish(rake.totalDistance)
        rake.battery.publish(rake.batteryLevel)
        rake.publisherCamera.publish(rake.activation_data)

        print("checkpoint3")
        #rake.random = rake.random + rake.vel.linear.x
        #print("[RANDOM DISTANCE]: {}".format(rake.random))
        print("[TOTAL DISTANCE]: {}".format(rake.totalDistance))
        print("[BATTERY LEVEL]: {}".format(rake.batteryLevel))
        print("[Twist Data]: {}".format(rake.vel))
        rake.Linear()
        rake.totalDistance = rake.length * rake.k + rake.length_among_rows * rake.l + rake.wentRoad
        rake.batteryLevel = 100 - (rake.totalDistance / 54)     

#after turn, publish only 90 or -90 data but assign 0 to the next turn.