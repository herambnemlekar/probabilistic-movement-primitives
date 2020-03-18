#!/usr/bin/env python

import adapy
import rospy
import numpy as np
import promp
import pickle

# Train the movement primitive model
# robot_model = promp.ProMP(10, 2, 8, "data")
# pickle.dump(robot_model, open("data/robot_model.p", "wb"))
# print "Trained"

# Randomly select a new object location
np.random.seed(1)
angle = np.random.uniform(np.pi/3, np.pi/2, 1)
lime_x = -0.65*np.sin(angle)[0]
lime_y = 0.65*np.cos(angle)[0]
lime_pose = np.matrix([1000*(lime_y + 0.025), 1000*(-lime_x - 0.527)])
print lime_x, lime_y, lime_pose

# Predict the robot trajectory
# prediction = robot_model.predictTraining()
robot_model = pickle.load(open("data/robot_model.p", "rb"))
prediction = robot_model.predictTest(lime_pose)
print "Predicted"

# Massage prediction into waypoints
pred = np.matrix(prediction).T
pred = np.matrix.tolist(pred[:, 2:8])
waypoints = []
for i, w in enumerate(pred):
    waypoints.append((float(i), w))

rospy.init_node("adapy_simple_traj")
rate = rospy.Rate(10)

if not rospy.is_shutdown():
    ada = adapy.Ada(False)
    executor = ada.start_trajectory_executor()
    print "HERE"

    # For simulation
    # viewer = ada.start_viewer("dart_markers/simple_trajectories", "map")
    # limeURDFUri = "package://pr_assets/data/objects/lime.urdf"
    # limePose = [lime_x, lime_y, 0.28, 0, 0, 0, 1]
    # boxURDFUri = "package://pr_assets/data/objects/large_block.urdf"
    # boxPose = [-0.7, 0.2, 0.0, 0.707107, 0, 0, 0.0]
    # world = ada.get_world()
    # lime = world.add_body_from_urdf(limeURDFUri, limePose)
    # box = world.add_body_from_urdf(boxURDFUri, boxPose)

    rospy.sleep(3.0)

    collision = ada.get_self_collision_constraint()

    arm_skeleton = ada.get_arm_skeleton()
    positions = arm_skeleton.get_positions()
    arm_state_space = ada.get_arm_state_space()

    # Arbitrary waypoints
    positions2 = positions.copy()
    positions2[1] -= 0.5
    positions2[2] += 0.2
    positions3 = [0., 3.14159265, 3.14159265, 0., 0., 0.]

    # Calculate trajectory directly to end point
    config = np.array([positions3])
    traj = ada.plan_to_configuration(arm_state_space, arm_skeleton, config.T)

    # Calculate trajectory for waypoints
    # waypoints = [(0.0, positions), (1.0, positions2), (2.0, positions3)]
    # traj = ada.compute_joint_space_path(arm_state_space, waypoints)
    ada.execute_trajectory(traj)

    # Grasping commands
    # hand = ada.get_hand()
    # press = np.array([0.75, 0.75])
    # hand.execute_preshape(press.T)

    rospy.sleep(7.0)
