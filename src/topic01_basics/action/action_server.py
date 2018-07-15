#! /usr/bin/env python

import rospy

from actionlib import SimpleActionServer

from riotu_ros_training.msg import FibonacciFeedback
from riotu_ros_training.msg import FibonacciResult
from riotu_ros_training.msg import FibonacciAction

class FibonacciActionServer(object):
    # create messages that are used to publish feedback/result
    feedback = FibonacciFeedback()
    result = FibonacciResult()

    def __init__(self, name):
        self.action_name = name
        self.action_server = SimpleActionServer(self.action_name, FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self.feedback.sequence = []
        self.feedback.sequence.append(0)
        self.feedback.sequence.append(1)
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self.action_name, goal.order, self.feedback.sequence[0], self.feedback.sequence[1]))
        
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self.action_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.action_name)
                self.action_server.set_preempted()
                success = False
                break
            self.feedback.sequence.append(self.feedback.sequence[i] + self.feedback.sequence[i-1])
            # publish the feedback
            rospy.loginfo('publishing feedback ...')
            self.action_server.publish_feedback(self.feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
            self.result.sequence = self.feedback.sequence
            rospy.loginfo('%s: Succeeded' % self.action_name)
            self.action_server.set_succeeded(self.result)
        
if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = FibonacciActionServer(rospy.get_name())
    rospy.spin()