#! /usr/bin/env python

import rospy
#import subprocess
#import sys
#import threading

import actionlib
from mongodb_openni_compression.msg import RecordCameraAction, RecordCameraResult, RecordCameraFeedback
from roslaunch_axserver.msg import launchAction, launchGoal, launchResult, launchFeedback

class RoslaunchServer(object):
# create messages that are used to publish feedback/result

    def __init__(self, name):
        self._action_name = name
        self.roslaunch_axclient = actionlib.SimpleActionClient('/launchServer', launchAction)
        rospy.loginfo("Waiting for roslaunch action...") 
        self.roslaunch_axclient.wait_for_server()
        rospy.loginfo("Done") 
        self.server = actionlib.ActionServer(name, RecordCameraAction, self.execute_cb, self.cancel_cb)
        self.server.start()
        rospy.loginfo("/record_camera action server started...")

    def cancel_cb(self, gh):
        self.roslaunch_axclient.cancel_all_goals()
        gh.set_canceled()

    def execute_cb(self, gh):
        gh.set_accepted()
        mygoal = gh.get_goal()
        goal = launchGoal()
        goal.pkg = "mongodb_openni_compression"
        goal.launch_file = "record.launch camera:=" + mygoal.camera
        if mygoal.with_depth:
            goal.launch_file = goal.launch_file + " with_depth:=true"
        if mygoal.with_rgb:
            goal.launch_file = goal.launch_file + " with_rgb:=true"
        print goal.pkg + " " + goal.launch_file
        self.roslaunch_axclient.send_goal(goal)
        #while (gh.get_goal_status().status == actionlib.GoalStatus.ACTIVE
               # and not rospy.is_shutdown()):
            #feedback = RecordCameraFeedback()
            #feedback.ready = self.roslaunch_axclient.get_state() == actionlib.GoalStatus.ACTIVE
            #gh.publish_feedback(feedback)
            #rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('record_camera')
    RoslaunchServer(rospy.get_name())
    rospy.spin()
