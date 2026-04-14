# -*- coding: utf-8 -*-
"""
Created on Tue Apr 14 12:58:33 2026

@author: gangu
"""

import pybullet as p
import pybullet_data
import time
import numpy as np

#-----------CLOSE PREVIOUSLY OPENED GUI LINKS---------------
try:
    p.disconnect()
except:
    pass
# ------------------ SETUP ------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Restore your camera
p.resetDebugVisualizerCamera(
    cameraDistance=3.5,
    cameraYaw=0,
    cameraPitch=-30,
    cameraTargetPosition=[0.55, -0.15, 0.2]
)

# ------------------ TABLE (WORKSPACE) ------------------

TABLE_CENTER = [1, 2, -0.65]

# Scale table so it spans grid [0.5 → 3.5]
table = p.loadURDF(
    "table/table.urdf",
    basePosition=TABLE_CENTER,
    globalScaling=2.5   # 🔥 important: makes table large enough
)

TABLE_Z = 0.9 # top surface approx

# ------------------ ROBOT ------------------

robot = p.loadURDF(
    "franka_panda/panda.urdf",
    basePosition=[-0.75, 2.0, TABLE_Z],  # slightly left → covers full table
    useFixedBase=True
)

ee_link = 11

# ------------------ GRID ON TABLE ------------------

grid_points = []
for i in range(3):
    for j in range(3):
        x = 0 + i * 1   # spacing adjusted to fill table nicely
        y = 1.2 + j * 0.5
        z = TABLE_Z
        grid_points.append([x, y, z])

# visualize grid
for idx, pt in enumerate(grid_points):
    p.addUserDebugText(str(idx), [pt[0], pt[1], pt[2] + 0.02],
                       textColorRGB=[1, 0, 0],
                       textSize=1.5)

# ------------------ CUBE (ON TABLE GRID) ------------------

cube_size = 0.1
half = cube_size / 2

start_idx = 0   # pick from grid[0]
goal_idx = 8    # place at grid[8]

cube = p.createMultiBody(
    baseMass=0.2,
    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[half]*3),
    baseVisualShapeIndex=p.createVisualShape(
        p.GEOM_BOX, halfExtents=[half]*3,
        rgbaColor=[0.2, 0.6, 0.9, 1]
    ),
    basePosition=[
        grid_points[start_idx][0],
        grid_points[start_idx][1],
        TABLE_Z + half
    ]
)

arm_start_position = [0,0.9,TABLE_Z+0.5]

#------- Function to move the robot arm--------
def move(target):
    current_pos = p.getLinkState(robot, ee_link)[0]

    for t in np.linspace(0, 1, 100):
        interp = (1 - t) * np.array(current_pos) + t * np.array(target)

        joint_poses = p.calculateInverseKinematics(
            robot,
            ee_link,
            interp
        )

        for i in range(7):
            p.setJointMotorControl2(
                robot,
                i,
                p.POSITION_CONTROL,
                joint_poses[i],
                force=800
            )

        p.stepSimulation()
        time.sleep(0.1)

#-------------------- RESET -----------------
move(arm_start_position)

# ------------------ MOTION ------------------

#-------------------PICK-UP--------------------
def pickup():
    # get end-effector position
    ee_pos = p.getLinkState(robot, ee_link)[0]

    # get cube position
    cube_pos, _ = p.getBasePositionAndOrientation(cube)

    # move above cube
    move([cube_pos[0], cube_pos[1], ee_pos[2]])

    # move down to cube
    move([cube_pos[0], cube_pos[1], cube_pos[2] + 0.02])

    # create fixed constraint (grasp)
    cid = p.createConstraint(
        parentBodyUniqueId=robot,
        parentLinkIndex=ee_link,
        childBodyUniqueId=cube,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )

    return cid

def release(cid):
    p.removeConstraint(cid)
    
cid = pickup()

# lift after grasp
move([grid_points[start_idx][0],
      grid_points[start_idx][1],
      TABLE_Z + 0.5])
#--------------- Check for if the cube is in the grid where the arm is stopped--------


# def move_ee(target_pos, steps=120):
#     current_pos = p.getLinkState(robot, ee_link)[0]

#     for i in range(steps):
#         alpha = i / steps
#         interp = (1 - alpha) * np.array(current_pos) + alpha * np.array(target_pos)

#         joint_poses = p.calculateInverseKinematics(robot, ee_link, interp)

#         for j in range(7):
#             p.setJointMotorControl2(
#                 robot, j,
#                 p.POSITION_CONTROL,
#                 joint_poses[j],
#                 force=800
#             )

#         p.stepSimulation()
#         time.sleep(1/240.)

# def grasp():
#     return p.createConstraint(
#         robot, ee_link,
#         cube, -1,
#         p.JOINT_FIXED,
#         [0,0,0],
#         [0,0,0],
#         [0,0,0]
#     )

# def release(cid):
#     p.removeConstraint(cid)

# # ------------------ PICK & PLACE ------------------

# APPROACH_Z = TABLE_Z + 0.3
# GRASP_Z = TABLE_Z + 0.06

# pick = grid_points[start_idx]
# place = grid_points[goal_idx]

# # Approach pick
# move_ee([pick[0], pick[1], APPROACH_Z])

# # Down
# move_ee([pick[0], pick[1], GRASP_Z])

# cid = grasp()

# # Lift
# move_ee([pick[0], pick[1], APPROACH_Z])

# # Move across table
# move_ee([place[0], place[1], APPROACH_Z])

# # Down
# move_ee([place[0], place[1], GRASP_Z])

# release(cid)

# # Retreat
# move_ee([place[0], place[1], APPROACH_Z])

# # ------------------ LOOP ------------------

# while True:
#     p.stepSimulation()
#     time.sleep(1/240.)