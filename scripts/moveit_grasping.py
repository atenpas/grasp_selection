#!/usr/bin/env python

import numpy
import sys
import copy

# ROS dependencies
import rospy
import moveit_commander
from moveit_msgs.msg import *
from moveit_msgs.srv import GetPlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import *
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# Baxter SDK dependencies
import baxter_interface

# dependencies on own packages
from grasp_selection.srv import *


def quittableInput(s):
  s = raw_input(s + "(q to quit) ")
  if s == 'q': exit(0)


def addObstacle():
  ## This does not work!
  ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot. (does not seem to work!)
  #scene = moveit_commander.PlanningSceneInterface()
  #scene.add_box("sensor_platform", PoseStamped(Point(0,0,0), Quaternion(0,0,0,1)), (1,1,1))
      
  #scene_pub = rospy.Publisher('/planning_scene', PlanningScene)
  #rospy.wait_for_service('/get_planning_scene', 10.0)
  #get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
  #request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
  #response = get_planning_scene(request)
  #coll_mat = response.scene.allowed_collision_matrix
  #
  #print "response.scene.world.collision_objects:", response.scene.world.collision_objects
  #
  #pose_box = Pose(Point(0,0,0), Quaternion(0,0,0,1))
  #box = SolidPrimitive()
  #box.type = SolidPrimitive.BOX
  #box.dimensions = [1,1,1]
  #coll_obj = CollisionObject()
  #coll_obj.header.stamp = rospy.Time(0)
  #coll_obj.header.frame_id = "/base"
  #coll_obj.id = "sensor_platform"
  #coll_obj.operation = CollisionObject.ADD
  #coll_obj.primitives.append(box)
  #coll_obj.primitive_poses.append(pose_box)
  #print "===== Coll Obj"
  #print coll_obj
    #
  #scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix = coll_mat)
  #scene_diff.world.collision_objects.append(coll_obj)
  #print "===== scene_diff"
  #print scene_diff
  #scene_pub.publish(scene_diff)
  #print "Publishing msg"
  #rospy.sleep(3)
  #
  #request = PlanningSceneComponents(components=16)
  #response = get_planning_scene(request)
  #coll_mat = response.scene.allowed_collision_matrix
  #
  #print "response.scene.world.collision_objects:", response.scene.world.collision_objects
  pass


#def changeTrajectorySpeed(traj, speeds): # this leads to weird behaviour
  #traj_new = RobotTrajectory()
  #traj_new = traj
  #
  #n = len(traj_new.joint_trajectory.points)
  #m = len(speeds)
  #indices = n/m * numpy.array(range(1,m+1))
  #indices[len(indices) - 1] = n
  #idx = 0
  #incr_old = traj_new.joint_trajectory.points[n-1].time_from_start / n
  #incr_new = incr_old / speeds[idx]
  #print "incr:", incr_old, " ", incr_new
  #print "indices where speed is changed:", indices, "n:", n, "m:", m
  #t = rospy.Duration(0)
  #
  #for i in range(0, len(traj_new.joint_trajectory.points)):
    #t += incr_new
    #traj_new.joint_trajectory.points[i].time_from_start = t
    #if m > 1 and i == indices[idx]:
      #idx += 1
      #sys.stdout.write("change incr from " + str(incr_new))
      #incr_new = incr_old / speeds[idx]
      #sys.stdout.write(" to " + str(incr_new) + "\n")
            #
  #return traj_new


def changeTrajectorySpeed(traj, speed):
  traj_new = RobotTrajectory()
  traj_new = traj
      
  for p in traj_new.joint_trajectory.points:
    p.time_from_start /= speed
                
  return traj_new


def generateWaypointsVertical(pose, z_target, num_waypoints = 5):
  waypoints = []
  waypoints.append(pose)
  wpose = copy.deepcopy(waypoints[0])
  dist_z = z_target - pose.position.z
  
  for i in range(0, num_waypoints):
    wpose.position.z += dist_z / num_waypoints
    waypoints.append(copy.deepcopy(wpose))
  
  return waypoints


def generateWaypointsGrasp(response, idx):
  num_waypoints = 3
  dist = [0.12, 0.06, 0.0] # Baxter gripper
  grasp = response.grasps.grasps[idx]
  waypoints = [None] * num_waypoints
      
  for i in range(0, num_waypoints):
    waypoints[i] = Pose()
    waypoints[i].position.x = grasp.pose.position.x - dist[i] * grasp.approach.x
    waypoints[i].position.y = grasp.pose.position.y - dist[i] * grasp.approach.y
    waypoints[i].position.z = grasp.pose.position.z - dist[i] * grasp.approach.z
    waypoints[i].orientation = grasp.pose.orientation
    print "---- waypoint", i, "----"
    print waypoints[i].position
  
  return waypoints


def createExtraWaypoint(pose, waypoint):
  way0 = waypoint.position
  way0 = numpy.array([way0.x, way0.y, way0.z])
  pos_curr = pose.position
  pos_curr = numpy.array([pos_curr.x, pos_curr.y, pos_curr.z])
  way_add = pos_curr + 0.5 * (way0 - pos_curr)
  pose = Pose()
  #pose.orientation = msg.orientation
  pose.orientation = waypoint.orientation
  pose.position = Point(way_add[0], way_add[1], way_add[2])
  return pose


def createPointMarker(waypoint, id):
  marker = Marker()
  marker.type = Marker.SPHERE
  marker.id = id
  marker.ns = "waypoints"
  marker.header.frame_id = "/base"
  marker.header.stamp = rospy.get_rostime()
  marker.lifetime = rospy.Duration.from_sec(60.0)
  marker.action = Marker.ADD
  marker.scale.x = marker.scale.y = marker.scale.z = 0.015
  marker.color.r = 1.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  marker.color.a = 1.0
  marker.pose.position = waypoint.position
  marker.pose.orientation = waypoint.orientation
  return marker


def createWaypointMarkers(waypoints):
  marker_array = MarkerArray()
  for i in range(0, len(waypoints)):
    marker_array.markers.append(createPointMarker(waypoints[i], i))
  return marker_array


def planAndExecuteWaypoints(group, waypoints, speeds = [0.6]):
  (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
  print "fraction:", fraction
  print "============ Waiting while RVIZ displays plan ..."
  rospy.sleep(2)
  
  if fraction < 0.8:
    print "Less than 80% of requested waypoints can be executed!"
    return False
  
  new_traj = None
  if len(speeds) == 1:
    new_traj = changeTrajectorySpeed(plan, speeds[0])
    quittableInput('Hit Enter to move arm ')
    group.execute(new_traj)
  else:
    for i in range(0, len(speeds)):
      (plan, fraction) = group.compute_cartesian_path(waypoints[i*2:i*2+2], 0.005, 0.0)
      new_traj = changeTrajectorySpeed(plan, speeds[i])
      quittableInput('Hit Enter to move arm ')
      group.execute(new_traj)  
  
  return True
  

def planAndExecuteJointTarget(group, joints, speed = 3.0):
  group.set_joint_value_target(joints)
  plan = group.plan()
  print "============ Waiting while RVIZ displays plan ..."
  rospy.sleep(2)
  new_traj = changeTrajectorySpeed(plan, speed)
  quittableInput('Hit Enter to move arm ')
  group.execute(new_traj)
  return True


def selectMostVerticalGrasp(response):
  z = numpy.array([0, 0, 1])
  max_dot = 0.0
  i = 0
  idx = -1
  for grasp in response.grasps.grasps:
    n = -1.0 * numpy.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
    dot = numpy.dot(n, z)
    if dot > max_dot:
      max_dot = dot
      idx = i
    i = i + 1
  print "Select grasp #", idx, "max_dot:", max_dot
  return idx


def createTargetMarker(response, idx):
  grasp = response.grasps.grasps[idx]
  diam = 0.01
  alpha = 1.0
  marker = Marker()
  marker.type = Marker.ARROW
  marker.id = 0
  marker.header.frame_id = "/base"
  marker.header.stamp = rospy.get_rostime()
  marker.lifetime = rospy.Duration.from_sec(60.0)
  marker.action = Marker.ADD
  marker.scale.x = diam # shaft diameter
  marker.scale.y = diam # head diameter
  marker.scale.z = 0.01 # head length
  marker.color.r = 1.0
  marker.color.g = 0.0
  marker.color.b = 0.0
  marker.color.a = alpha
  p = Point()
  q = Point()
  p.x = grasp.pose.position.x
  p.y = grasp.pose.position.y
  p.z = grasp.pose.position.z
  q.x = p.x - 0.15 * grasp.approach.x
  q.y = p.y - 0.15 * grasp.approach.y
  q.z = p.z - 0.15 * grasp.approach.z
  marker.points.append(p)
  marker.points.append(q)
  return marker


def grasping():
  # initialize MoveIt and ROS
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('moveit_grasping')
  
  # create publisher to request point clouds
  cloud_pub = rospy.Publisher('cloud_request', String, queue_size=10)
    
  # create publisher to visualize the grasp target
  target_pub = rospy.Publisher('target', Marker, queue_size=1)
    
  # create publisher to visualize the waypoints
  waypoints_pub = rospy.Publisher('waypoints', MarkerArray, queue_size=1)
  
  # wait for ROS service for grasp selection
  rospy.wait_for_service('/select_grasps/select_grasps')
  select_grasps = rospy.ServiceProxy('/select_grasps/select_grasps', SelectGrasps)
  
  # initialize robot hand
  right_hand = baxter_interface.Gripper('right', baxter_interface.CHECK_VERSION)

  ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
  robot = moveit_commander.RobotCommander()
      
  ## Instantiate a MoveGroupCommander object.  This object is an interface to one group of joints.
  group = moveit_commander.MoveGroupCommander("right_arm")
  group.stop()

  print "============ Reference frame: %s" % group.get_planning_frame()
  print "============ Reference frame: %s" % group.get_end_effector_link()
  print "============ Robot Groups:"
  print robot.get_group_names()
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============ Printing joint positions"
  print group.get_joints()
  print group.get_current_joint_values()
  print "============ Printing current robot hand pose"
  pose_curr = group.get_current_pose().pose
  
  # calibrate robot hand
  if not right_hand.calibrated():
    instr = raw_input("Calibrate gripper (Y/n): ")
    if instr == "y" or instr == "Y":
      right_hand.calibrate()
  
  # set max forces
  right_hand.set_holding_force(100)
  right_hand.set_moving_force(100)
  
  # open robot hand
  right_hand.open(block=True)
  
  # move robot hand up if it is too low
  if pose_curr.position.z < 0.20:
    planAndExecuteWaypoints(group, generateWaypointsVertical(group.get_current_pose().pose, 0.20), [5.0])

  # define joint targets
  joints_pre_nominal = [0.017640779040527344, -1.074170045489502, 0.39269908125, 2.052466291845703, -0.4233786969726563, 0.6879903825805664, 0.031446606115722656]
  joints_nominal = [0.6469563965515137, -0.8912428367431641, -0.03298058690185547, 2.2940682656616214, -0.22319420438232423, -0.6887573729736328, 0.08590292402343751]
    
  while not rospy.is_shutdown():
    # move to pre-nominal joint target
    planAndExecuteJointTarget(group, joints_pre_nominal, 1.6)
    rospy.sleep(0.1)
    
    # open the robot hand
    right_hand.open()
            
    # request point cloud
    quittableInput("Hit Enter to request point cloud (q to quit) ")
    cloud_pub.publish("request_cloud")
    rospy.sleep(3.0) # wait for point cloud to arrive
    
    # move to nominal joint target
    planAndExecuteJointTarget(group, joints_nominal, 1.6)
    rospy.sleep(0.1)
    
    # request grasps from grasp selection node
    has_grasps = False
    resp = None
    while not has_grasps:
      quittableInput("Hit Enter to request grasps ")
      try:
        resp = select_grasps(group.get_current_pose().pose)
        has_grasps = True          
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        break    
    if not has_grasps:
      continue
    
    # select most vertical grasp (best top grasp) and visualize it in Rviz
    idx = selectMostVerticalGrasp(resp)
    target_pub.publish(createTargetMarker(resp, idx))
    print "Selected grasp #", idx
    rospy.sleep(0.1)
    
    # generate waypoints
    waypoints = generateWaypointsGrasp(resp, idx)
    print "Generated waypoints"
          
    # create extra waypoint in between current joint positions and first grasp waypoint      
    waypoints.insert(0, createExtraWaypoint(group.get_current_pose().pose, waypoints[0]))
    
    # visualize waypoints
    waypoints_pub.publish(createWaypointMarkers(waypoints))
    rospy.sleep(0.1)
    
    # add current robot hand pose to waypoints
    waypoints.insert(0, group.get_current_pose().pose)
    
    # move to grasp waypoints
    print ""
    print "==== Grasp Waypoints ==="
    print waypoints
    planAndExecuteWaypoints(group, waypoints, [0.7])
    #planAndExecuteWaypoints(group, waypoints, [2.0, 1.0, 0.7])
    
    # close the robot hand
    print "Reached grasp target"
    quittableInput("Hit Enter to close the hand")
    right_hand.close()
    rospy.sleep(1.0)
    
    # lift grasped object 10cm up
    planAndExecuteWaypoints(group, generateWaypointsVertical(group.get_current_pose().pose, 0.0), [1.0])
    rospy.sleep(1.0)
    
    # open the robot hand
    quittableInput("Hit Enter to open the hand")
    right_hand.open()
    rospy.sleep(1.0)
    

if __name__=='__main__':
  try:
    grasping()
  except rospy.ROSInterruptException:
    pass
