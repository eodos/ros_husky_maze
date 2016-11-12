#! /usr/bin/env python

import rospy                                          
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse
import time
import math

class HuskyService():
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.callback_odom)
        self.imu_sub = rospy.Subscriber('/sphero/imu/data', Imu, self.callback_imu)
        self.laser_sub = rospy.Subscriber('/camera/scan', LaserScan, self.callback_laser)
        self.exit_service = rospy.Service("/husky_exit", Trigger , self.callback_service)
      
        self.imu_data = Imu()
        self.odometry_data = Odometry()
        self.laser_data = LaserScan()
        self.init_distance = 0
        self.first_time = True

    def callback_service(self, request):
        self.response = TriggerResponse()
        self.response.success = False
        self.response.message = "None"
        
        if self.first_time:
            self.first_time = False
            x = self.odometry_data.pose.pose.position.x
            y = self.odometry_data.pose.pose.position.y
            self.init_distance = math.sqrt(x**2 + y**2)
        
        middle_laser = len(self.laser_data.ranges)/2
        # rospy.loginfo('Middle laser: ' + str(self.laser_data.ranges[middle_laser]))
        
        # for i in range(len(self.laser_data.ranges)):
        #     if self.laser_data.ranges[i] > 25:
        #         if i < len(self.laser_data.ranges)/2:
        #             self.response.message = "RIGHT"
        #             self.response.success = True
        #             break
        #         elif i > len(self.laser_data.ranges)/2:
        #             self.response.message = "LEFT"
        #             self.response.success = True
        #             break
        #         else:
        #             self.response.message = "FRONT"
        #             self.response.success = True
        
        #Check laser data is not empty
        if middle_laser > 2:
        
            if self.laser_data.ranges[middle_laser] > 25:
                self.response.message = "FRONT"
            elif self.laser_data.ranges[0] > 25:
                self.response.message = "RIGHT"
            elif self.laser_data.ranges[len(self.laser_data.ranges)-1] > 25:
                self.response.message = "RIGHT"
                
            # Now, check completion using odometry data
            x = self.odometry_data.pose.pose.position.x
            y = self.odometry_data.pose.pose.position.y
            distance = math.sqrt(x**2 + y**2)
            if distance - self.init_distance > 6:
                rospy.logwarn('OUTSIDE MAZE')
                self.response.success = True
            
        return self.response
       
    def callback_odom(self, msg):
        self.odometry_data = msg
        
    def callback_imu(self, msg):
        self.imu_data = msg
        
    def callback_laser(self, msg):
        self.laser_data = msg

if __name__ == "__main__":
    rospy.init_node('husky_service')
    time.sleep(1)
    HuskyService()
    rospy.spin()
