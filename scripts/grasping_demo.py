#!/usr/bin/env python

# system dependencies
import copy
import csv
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

# Baxter SDK dependencies
import baxter_interface
from baxter_core_msgs.msg import JointCommand

# OpenRave
import openravepy

# dependencies on own packages
from grasp_selection.srv import *


class Grasping():
  """
  Class for a simple grasping demo. 
  Requests grasps from the grasp_selection package and randomly selects one of the grasps given as a response by that 
  package. Also contains a velocity controller for moving the robot arm.
  """

  def __init__(self):
    """
    """
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

  
  def moveVelocity(self, joint_values_target, speed, thresh, is_printing = False):
    """
    Velocity controller for Baxter arm movements.
    """
    max_iterations = 5000
    step_size = 2.0
  
    for i in range(0, max_iterations):
      joint_error = numpy.array(self.joint_values)
      joint_velocity_command = joint_error
    
      # calculate joint error, error magnitude, and command velocity
      joint_error = joint_values_target - joint_error
      joint_velocity_command = joint_error * step_size
      error_magnitude = numpy.linalg.norm(joint_error)
      joint_velocity_command_magnitude = error_magnitude * step_size
      
      # if command velocity magnitude exceeds speed, scale it back to speed
      if joint_velocity_command_magnitude > speed:
        joint_velocity_command = speed * joint_velocity_command / joint_velocity_command_magnitude
      
      # send velocity command to joints
      values_out = [joint_velocity_command[2], joint_velocity_command[3], joint_velocity_command[0], 
        joint_velocity_command[1], joint_velocity_command[4], joint_velocity_command[5], joint_velocity_command[6]]
      self.pub.publish(JointCommand(JointCommand.VELOCITY_MODE, values_out, self.joint_names))
      
      if is_printing:
        print "iteration:", i, "error_magnitude: ", error_magnitude
      
      # break loop if close enough to target joint values
      if error_magnitude < thresh:
        break
      
      rospy.sleep(0.001)
  
    # need to reset to position mode; o.w. the robot will disable itself
    values_out = [joint_values_target[2], joint_values_target[3], joint_values_target[0], joint_values_target[1], 
      joint_values_target[4], joint_values_target[5], joint_values_target[6]]
    self.pub.publish(JointCommand(JointCommand.POSITION_MODE, values_out, self.joint_names))
    
  
  def transformToGeometryMsg(self, pose_mat):
    """
    Convert a transformation matrix to a geometry_msgs::Pose ROS message.
    """
    msg = Pose()
    quat = tf.transformations.quaternion_from_matrix(pose_mat)
    msg.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    msg.position = Point(pose_mat[0,3], pose_mat[1,3], pose_mat[2,3])
    return msg
  
  
  def geometryMsgToTransform(self, msg):
    """
    Convert a geometry_msgs::Pose ROS message to a rotation matrix.
    """
    mat = tf.transformations.quaternion_matrix([msg.orientation.x, msg.orientation.y, msg.orientation.z, 
      msg.orientation.w])
    mat[0:3,3] = [msg.position.x, msg.position.y, msg.position.z]
    return mat
    
  
  def run(self):
    """
    Run the grasping demo.
    """
    
    # initialize ROS
    print "Initializing ROS"
    rospy.init_node('grasping_demo')
    
    print "Initializing OpenRave"
    env = openravepy.Environment()
    env.StopSimulation()
    openrave_root = '/home/baxter/baxter_ws/src/grasp_selection/openrave/'
    env.Load(openrave_root + "manipulator_baxter_structure.xml")
    env.Load(openrave_root + "table.xml")
    
    # initialize the random number generator
    random.seed()
              
    # wait for ROS service for grasp selection
    rospy.wait_for_service('/select_grasps/select_grasps')
    select_grasps = rospy.ServiceProxy('/select_grasps/select_grasps', SelectGrasps)

    # create publisher to request point clouds
    pub = rospy.Publisher('cloud_request', String, queue_size=10)
        
    # create publisher to move right arm
    self.pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
    
    # initialize robot hand
    right_hand = baxter_interface.Gripper('right', baxter_interface.CHECK_VERSION)
    
    # calibrate robot hand
    if not right_hand.calibrated():
      instr = raw_input("Calibrate gripper (Y/n): ")
      if instr == "y" or instr == "Y":
        right_hand.calibrate()
    
    # open robot hand
    right_hand.open(block=True)
    
    joints_pre_nominal = [0.017640779040527344, -1.074170045489502, 0.39269908125, 2.052466291845703, -0.4233786969726563, 0.6879903825805664, 0.031446606115722656]
    joints_nominal = [0.4996942410827637, -0.8034224367370606, 0.18906313189086915, 2.178252716308594, -0.08436894323730469, -0.6358350358520508, -0.10699515983276368]
              
    env.SetViewer('qtcoin') # start the viewer (conflicts with matplotlib)
    
    robot = env.GetRobots()[0] # get the first robot
    
    manip = robot.SetActiveManipulator('right_arm') # set the manipulator to right_arm
    self.joint_indices = manip.GetArmIndices()
    self.joints_sub = rospy.Subscriber("/robot/joint_states", sensor_msgs.JointState, self.jointsCallback)
    rospy.sleep(3)
    
    # move arm upward if too low
    robot.SetDOFValues(self.joint_values, manip.GetArmIndices()) # set the current solution
    env.UpdatePublishedBodies() # allow viewer to update new robot
    current_pose = manip.GetEndEffectorTransform()
    
    if current_pose[2,3] < 0:
      goal = copy.deepcopy(current_pose)
      goal[2,3] = 0.15
      print "goal:", goal
      sol = manip.FindIKSolution(goal, openravepy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
      print "IK solution:", sol
      robot.SetDOFValues(sol, manip.GetArmIndices()) # set the current solution
      Tee = manip.GetEndEffectorTransform()
      env.UpdatePublishedBodies() # allow viewer to update new robot
      print "FK:", Tee
      rospy.sleep(2)      
      s = raw_input("Hit Enter to move arm")
      self.moveVelocity(sol, 1, 0.1)
      rospy.sleep(0.5)

    while not rospy.is_shutdown():    
      # move to pre-nominal joint positions
      s = raw_input("Hit Enter to move arm")
      self.moveVelocity(joints_pre_nominal, 1, 0.05)
      robot.SetDOFValues(self.joint_values, manip.GetArmIndices()) # set the current solution
      env.UpdatePublishedBodies() # allow viewer to update
      rospy.sleep(0.1)
      
      # request point cloud
      s = raw_input("Hit Enter to request point cloud ")
      pub.publish("request_cloud")
      rospy.sleep(3.0) # wait for point cloud to arrive
      
      # open the robot hand
      right_hand.open()
      
      # move to nominal joint positions
      self.moveVelocity(joints_nominal, 1, 0.05)
      robot.SetDOFValues(self.joint_values, manip.GetArmIndices())
      env.UpdatePublishedBodies() # allow viewer to update
      rospy.sleep(0.1)
                      
      # request grasps from grasp selection node
      s = raw_input("Hit Enter to request grasps ")
      try:
        pose_msg = self.transformToGeometryMsg(manip.GetEndEffectorTransform())
        self.resp = select_grasps(pose_msg)
        
        # select grasp at random
        idx = random.randint(0, len(self.resp.grasps.grasps) - 1)
        print "Randomly selected grasp #", idx
                            
        # move arm to selected grasp
        num_waypoints = 3
        dist = [0.1, 0.05, 0.0]
        speeds = [0.8, 0.3, 0.3]
        threshs = [0.08, 0.03, 0.03]
        waypoints = [None] * num_waypoints
        for i in range(0,num_waypoints):
          wpose = copy.deepcopy(self.resp.grasps.grasps[idx].pose)
          wpose.position.x = wpose.position.x - dist[i] * self.resp.grasps.grasps[idx].approach.x
          wpose.position.y = wpose.position.y - dist[i] * self.resp.grasps.grasps[idx].approach.y
          wpose.position.z = wpose.position.z - dist[i] * self.resp.grasps.grasps[idx].approach.z
          waypoints[i] = copy.deepcopy(wpose)    
          print "---- waypoint", i, "----"
          print waypoints[i].position
        
        # move arm to waypoints
        ik_failed = False
        for i in range(0,num_waypoints):
          robot.SetDOFValues(self.joint_values, manip.GetArmIndices())
          goal = self.geometryMsgToTransform(waypoints[i])
          sol = manip.FindIKSolution(goal, openravepy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
          if sol == None:
            print "IK solver failed!"
            ik_failed = True
            break
          print "IK solution:", sol
          robot.SetDOFValues(sol, manip.GetArmIndices()) # set the current solution
          env.UpdatePublishedBodies() # allow viewer to update new robot
          self.moveVelocity(sol, speeds[i], threshs[i])
          rospy.sleep(0.5)
          
        # go back to prenominal pose if IK failed
        if ik_failed:
          s = raw_input("Do you want to (c)ontinue or (q)uit? ")
          if s == "q":
            break
          print "Going back to nominal pose"
          continue
          
        # close the robot hand
        print "Reached grasp target"
        right_hand.close()
        
        # lift grasped object 10cm up
        goal = copy.deepcopy(manip.GetEndEffectorTransform())
        goal[2,3] = goal[2,3] + 0.1            
        sol = manip.FindIKSolution(goal, openravepy.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
        robot.SetDOFValues(sol, manip.GetArmIndices()) # set the current solution            
        env.UpdatePublishedBodies() # allow viewer to update new robot
        s = raw_input("Hit Enter to move arm")
        self.moveVelocity(sol, 1, 0.1)
        rospy.sleep(0.1)
                    
        # open the robot hand
        s = raw_input("Hit Enter to open the hand ")
        right_hand.open()
        
        s = raw_input("Do you want to (c)ontinue or (q)uit? ")
        if s == "q":
          break
                  
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e
  
    exit()


# main code
if __name__ == '__main__':
  try:
    grasp = Grasping()
    grasp.run()
  except rospy.ROSInterruptException:
    pass
