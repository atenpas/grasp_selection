#!/usr/bin/env python

import openravepy

env = openravepy.Environment()
env.StopSimulation()
#env.Load('/home/baxter/baxter_ws/src/sensor_explorer/openravestuff/manipulator_baxter_structure.xml')
openrave_root = '/home/baxter/baxter_ws/src/grasp_selection/openrave/'
env.Load(openrave_root + "manipulator_baxter_structure.xml")
