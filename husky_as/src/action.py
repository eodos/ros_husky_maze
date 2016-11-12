#! /usr/bin/env python

import rospy                                          
from nav_msgs.msg import Odometry
from husky_action_msg.msg import CustomOdometryAction, CustomOdometryResult
import actionlib

class HuskyActionServer():
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.callback_odom)
        self._as = actionlib.SimpleActionServer("/husky_as", CustomOdometryAction, self.callback_action, False)
        self._as.start()

        self.odometry_data = Odometry()
        
    def callback_action(self, goal):
        seconds_recording = 120
        r = rospy.Rate(1)
        success = True
        result = CustomOdometryResult()
        
        rospy.loginfo('Starting to log odometry data...')
        
        for _ in range(seconds_recording):
            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                self._as.set_preempted()
                success = False
                break
            
            result.result_odom_array.append(self.odometry_data)
            r.sleep()
    
        if success:
            rospy.loginfo('Finished logging odometry data.')
            self._as.set_succeeded(result)
        
 
    def callback_odom(self, msg):
        self.odometry_data = msg

if __name__ == "__main__":
    rospy.init_node('husky_as')
    HuskyActionServer()
    rospy.spin()
