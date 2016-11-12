#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from husky_action_msg.msg import CustomOdometryAction, CustomOdometryGoal
import actionlib
import time

class HuskyCommander():
    
    def __init__(self):
        self.p = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.wait_for_service('/husky_exit')
        self.laser_service = rospy.ServiceProxy('/husky_exit', Trigger)
        
        self.goal = CustomOdometryGoal()
        self.action_client = actionlib.SimpleActionClient('/husky_as', CustomOdometryAction)
        self.action_client.wait_for_server()
        self.action_client.send_goal(self.goal, feedback_cb=self.callback_action)
        self.state_result = 0
        
        self.rate = rospy.Rate(10)
        self.velocities = Twist()
        
    def callback_action():
        pass
        
    def move_forward(self):
        self.set_speed(0.5, 0)
        
    def move_backward(self):
        self.set_speed(-0.5, 0)
        
        
    def turn_left(self):
        self.set_speed(0, 0.25)
        
    def turn_right(self):
        self.set_speed(0, -0.25)
    
    def stop(self):
        self.set_speed(0, 0)
        
    def set_speed(self, linear_x, angular_z):
        self.velocities.linear.x = linear_x
        self.velocities.angular.z = angular_z
        self.p.publish(self.velocities)

    def main(self):
        self.state_result = self.action_client.get_state()
        while self.state_result < 2:
            result = self.laser_service()
            # Check if we are outside the maze, if not, move
            if not result.success:
                if result.message == "FRONT":
                    rospy.loginfo('FOUND! - Moving forward')
                    self.move_forward()
                elif result.message == "LEFT":
                    rospy.loginfo('FOUND! - Turning to the left')
                    self.turn_left()
                elif result.message == "RIGHT":
                    rospy.loginfo('FOUND! - Turning to the right')
                    self.turn_right()
                else:
                    rospy.loginfo('Searching...')
                    self.turn_right()
            else:
                # Outside maze
                break
                
            self.rate.sleep()
            
            self.state_result = self.action_client.get_state()
        
        # print self.state_result
        # print self.action_client.get_result()
        # if self.state_result == 2 or self.state_result == 3:
        #     print 'TIME OVER'
        # else:
        #     print 'ERROR'
            
        self.stop()
        time.sleep(1)

if __name__ == "__main__":
    rospy.init_node('husky_commander')
    c = HuskyCommander()
    # Wait until we receive some laser packets
    time.sleep(1)
    c.main()
