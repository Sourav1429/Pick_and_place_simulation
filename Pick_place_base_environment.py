# -*- coding: utf-8 -*-
"""
Created on Thu Apr 16 12:40:38 2026

@author: gangu
"""

import pybullet as p
import pybullet_data
import time
import numpy as np
import random


class RobotEnv:
    def __init__(self):
        # ---------- CONNECTION ----------
        try:
            p.disconnect()
        except:
            pass

        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # ---------- CAMERA ----------
        p.resetDebugVisualizerCamera(
            cameraDistance=3.5,
            cameraYaw=0,
            cameraPitch=-89,
            cameraTargetPosition=[0.45, 0.5, 0.2]
        )

        # ---------- TABLE ----------
        self.TABLE_CENTER = [1, 2, -0.65]
        self.TABLE_Z = 0.9

        self.table = p.loadURDF(
            "table/table.urdf",
            basePosition=self.TABLE_CENTER,
            globalScaling=2.5
        )

        # ---------- ROBOT ----------
        self.robot = p.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[-0.75, 2.0, self.TABLE_Z],
            useFixedBase=True
        )

        self.ee_link = 11

        # ---------- GRID ----------
        self.grid_points = []
        for i in range(3):
            for j in range(3):
                x = -0.5 + i * 0.3
                y = 1.5 + j * 0.5
                z = self.TABLE_Z
                self.grid_points.append([x, y, z])

        for idx, pt in enumerate(self.grid_points):
            p.addUserDebugText(
                str(idx),
                [pt[0], pt[1], pt[2] + 0.02],
                textColorRGB=[1, 0, 0],
                textSize=1.5
            )

        # ---------- CUBE ----------
        self.cube_size = 0.1
        self.half = self.cube_size / 2

        self.cube = p.createMultiBody(
            baseMass=0.2,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[self.half]*3
            ),
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[self.half]*3,
                rgbaColor=[0.2, 0.6, 0.9, 1]
            ),
            basePosition=[
                self.grid_points[0][0],
                self.grid_points[0][1],
                self.TABLE_Z + self.half
            ]
        )

        self.arm_start_position = self.grid_points[0]

    # ---------- RANDOM CUBE ----------
    def place_cube_random(self):
        idx = random.randint(0, len(self.grid_points) - 1)

        x, y, _ = self.grid_points[idx]

        p.resetBasePositionAndOrientation(
            self.cube,
            [x, y, self.TABLE_Z + self.half],
            [0, 0, 0, 1]
        )

        return idx

    # ---------- MOVE ----------
    def move(self, target):
        current_pos = p.getLinkState(self.robot, self.ee_link)[0]

        for t in np.linspace(0, 1, 100):
            interp = (1 - t) * np.array(current_pos) + t * np.array(target)

            joint_poses = p.calculateInverseKinematics(
                self.robot,
                self.ee_link,
                interp
            )

            for i in range(7):
                p.setJointMotorControl2(
                    self.robot,
                    i,
                    p.POSITION_CONTROL,
                    joint_poses[i],
                    force=800
                )

            p.stepSimulation()
            time.sleep(0.1)

    # ---------- ALIGNMENT CHECK ----------
    def is_ee_over_cube(self, threshold=0.05):
        ee_pos = p.getLinkState(self.robot, self.ee_link)[0]
        cube_pos, _ = p.getBasePositionAndOrientation(self.cube)

        dist = np.linalg.norm([
            ee_pos[0] - cube_pos[0],
            ee_pos[1] - cube_pos[1]
        ])

        print("Distance:", dist)
        return dist < threshold

    # ---------- PICKUP ----------
    def pickup(self):
        cube_pos, _ = p.getBasePositionAndOrientation(self.cube)
        ee_pos = p.getLinkState(self.robot, self.ee_link)[0]

        self.move([cube_pos[0], cube_pos[1], ee_pos[2]])
        self.move([cube_pos[0], cube_pos[1], cube_pos[2] + 0.02])

        cid = p.createConstraint(
            self.robot,
            self.ee_link,
            self.cube,
            -1,
            p.JOINT_FIXED,
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        )

        return cid

    def release(self, cid):
        p.removeConstraint(cid)

    # ---------- RESET ----------
    def reset(self):
        idx = self.place_cube_random()
        self.move(self.arm_start_position)
        return self.grid_points[idx],self.arm_start_position

    # ---------- STEP EXAMPLE ----------
    def run(self):
        idx = self.reset()

        if self.is_ee_over_cube():
            cid = self.pickup()

            self.move([
                self.grid_points[0][0],
                self.grid_points[0][1],
                self.TABLE_Z + 0.5
            ])
        else:
            print("Negative reward")

        while True:
            p.stepSimulation()
            time.sleep(1/240.)
env = RobotEnv()
cube_pos,gripper_position = env.reset()
print("CUBE location:",cube_pos)
print("Robot arm gripper location:",gripper_position)