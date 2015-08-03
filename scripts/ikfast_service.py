#!/usr/bin/env python

# system dependencies
import copy
import math
import numpy
import random
import sys

# ROS dependencies
import rospy
import moveit_msgs.msg
from geometry_msgs.msg import *
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryAction
import cv
import sensor_msgs.msg as sensor_msgs
import tf
from visualization_msgs.msg import Marker, MarkerArray

# Baxter SDK dependencies
import baxter_interface
from baxter_core_msgs.msg import JointCommand

# OpenRave
import openravepy

# dependencies on own packages
from grasp_selection.srv import SolveIK, SolveIKResponse


class IKService():
  """
  Class for solving Inverse Kinematics. Communication is handled through a ROS service. 
  """

  def __init__(self):
    self.joint_values = [0] * 7
        
  
  def jointsCallback(self, msg):
    """
    Callback function for the joint_states ROS topic.
    """
    joint_values = msg.position[self.joint_indices[0] : self.joint_indices[6] + 1]
    self.joint_names = msg.name[self.joint_indices[0] : self.joint_indices[6] + 1]
    
    # ordering of jointstates topic is different from IK solver ordering
    self.joint_values[0] = joint_values[2]
    self.joint_values[1] = joint_values[3]
    self.joint_values[2] = joint_values[0]
    self.joint_values[3] = joint_values[1]
    self.joint_values[4:8] = joint_values[4:8]    
   
  
  def initOpenRave(self):
    self.env = openravepy.Environment()
    self.env.StopSimulation()
    openrave_root = '/home/baxter/baxter_ws/src/grasp_selection/openrave/'
    self.env.Load(openrave_root + "manipulator_baxter_structure.xml")
    self.env.Load(openrave_root + "table.xml")
    self.robot = self.env.GetRobots()[0] # get the first robot
    self.manip = self.robot.SetActiveManipulator('right_arm') # set the manipulator to right_arm
    self.joint_indices = self.manip.GetArmIndices()


  def findClosestIK(self, curr, sols):
    min_norm = 1000000
    best_sol = []
    for sol in sols:
      norm = numpy.linalg.norm(sol - curr)
      print "norm: ", norm, "sol:", sol
      if norm < min_norm:
        min_norm = norm
        best_sol = sol
    #print "curr:", curr
    print "best_sol:", best_sol
    #print "min_norm: ", min_norm
    return best_sol
 
 
  def geometryMsgToTransform(self, msg):
    """
    Convert a geometry_msgs::Pose ROS message to a rotation matrix (= numpy array).
    """
    mat = tf.transformations.quaternion_matrix([msg.orientation.x, msg.orientation.y, msg.orientation.z, 
      msg.orientation.w])
    mat[0:3,3] = [msg.position.x, msg.position.y, msg.position.z]
    return mat
  
  
  def handleSolveIKFast(self, req):    
    self.robot.SetDOFValues(self.joint_values, self.manip.GetArmIndices()) # set the current solution    
    sols = self.manip.FindIKSolutions(self.geometryMsgToTransform(req.target_pose), 18) # solve IK (ignore collisions)
    resp = SolveIKResponse()
    if len(sols) == 0:
      resp.solution = []    
      resp.success = False
    else:
      resp.solution = self.findClosestIK(self.joint_values, sols) # find closest IK solution
      resp.success = True
    rospy.loginfo( "---- Response send ----")
    return resp
    
  
  def run(self):     
    # initialize ROS
    print "Initializing ROS"
    rospy.init_node('ikfast_service')
           
    # initialize OpenRave
    print "Initializing OpenRave"
    self.initOpenRave()
    
    service = rospy.Service('ikfast_solver', SolveIK, self.handleSolveIKFast)
        
    # initialize the random number generator
    random.seed()
    
    # subscribe to joint_states ROS topic
    self.joints_sub = rospy.Subscriber("/robot/joint_states", sensor_msgs.JointState, self.jointsCallback)
    
    # run loop  
    print "Ready to receive Inverse Kinematics requests"
    rospy.spin()


# main code
if __name__ == '__main__':
  serv = IKService()
  serv.run()
