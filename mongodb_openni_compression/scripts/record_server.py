#! /usr/bin/env python

import rospy
#import subprocess
#import sys
#import threading

import actionlib
from actionlib_msgs.msg import GoalStatus
from mongodb_openni_compression.msg import RecordCameraAction, RecordCameraResult, RecordCameraFeedback
from roslaunch_axserver.msg import launchAction, launchGoal, launchResult, launchFeedback

class RecordServer(object):
# create messages that are used to publish feedback/result

    def __init__(self, name):
        self._action_name = name
        self.roslaunch_axclient = actionlib.SimpleActionClient('/record_roslaunch_axserver', launchAction)
        rospy.loginfo("Waiting for roslaunch action...") 
        self.roslaunch_axclient.wait_for_server()
        rospy.loginfo("Done") 
        self.server = actionlib.ActionServer(name, RecordCameraAction, self.execute_cb, self.cancel_cb)
        self.server.start()
        rospy.loginfo("/record_camera action server started...")

    def cancel_cb(self, gh):
        rospy.loginfo('cancel camera record goal ' + gh.get_goal_id().id)
        self.roslaunch_axclient.cancel_all_goals()
        gh.set_canceled()
        self.gh = None
        
    def feedback_cb(self, feedback):
        myfeedback = RecordCameraFeedback()
        myfeedback.ready = feedback.ready
        self.gh.publish_feedback(feedback)
        
    def done_cb(self, goal_status, result):
        if self.gh is None:
            return
        if goal_status == GoalStatus.SUCCEEDED:
            self.gh.set_succeeded()
        else:
            self.gh.set_aborted()
        self.gh = None
        
    def execute_cb(self, gh):
        gh.set_accepted()
        self.gh = gh
        mygoal = gh.get_goal()
        goal = launchGoal()
        goal.pkg = "mongodb_openni_compression"
        goal.launch_file = "record.launch camera:=" + mygoal.camera
        if not mygoal.with_depth:
            goal.launch_file = goal.launch_file + " with_depth:=false"
        if not mygoal.with_rgb:
            goal.launch_file = goal.launch_file + " with_rgb:=false"
        #goal.monitored_topics = ["/" + mygoal.camera + "head_xtion/rgb/image_raw", "/" + mygoal.camera + "head_xtion/depth/image_raw"]
        print goal.pkg + " " + goal.launch_file
        self.roslaunch_axclient.send_goal(goal, done_cb = self.done_cb, active_cb = None, feedback_cb = self.feedback_cb)

if __name__ == '__main__':
    rospy.init_node('record_camera')
    RecordServer(rospy.get_name())
    rospy.spin()
