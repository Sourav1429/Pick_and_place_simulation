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
    def __init__(self,n=3):
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
        self.n = n
        self.TABLE_CENTER = [1, 2, -0.65]
        self.TABLE_Z = 0.9
        self.cid = None
        self.status = "Not Holding"

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
        self.grid_text_id = -1

        # ---------- GRID ----------
        self.grid_points = []
        for i in range(self.n):
            for j in range(self.n):
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
                p.GEOM_BOX, halfExtents=[self.half]*self.n
            ),
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[self.half]*self.n,
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
    def show_current_grid(self):
        status = "Holding" if self.object_holding() else "Not Holding"

        if self.grid_text_id != -1:
            p.removeUserDebugItem(self.grid_text_id)
    
        ee_pos = p.getLinkState(
            self.robot,
            self.ee_link
        )[0]
    
        self.grid_text_id = p.addUserDebugText(
            f"Current Grid: {self.arm_position_idx}| Status:{status} the object",
            [ee_pos[0], ee_pos[1], ee_pos[2]+0.25],
            textColorRGB=[0,1,0],
            textSize=1.8
        )
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
    def is_ee_over_cube(self, threshold=0.15):
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
        self.move([cube_pos[0], cube_pos[1], cube_pos[2] + 0.005])

        self.cid = p.createConstraint(
            self.robot,
            self.ee_link,
            self.cube,
            -1,
            p.JOINT_FIXED,
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        )

    def release(self):
        print("Release called")
        if self.cid is not None:
    
            p.removeConstraint(self.cid)
    
            self.cid = None
    
            for _ in range(100):
                p.stepSimulation()
                time.sleep(1/240.)
    #--------- Holding object -----------------
    def object_holding(self):
        return self.cid is not None

    # ---------- RESET ----------
    def reset(self):
        idx = self.place_cube_random()
        self.move(self.arm_start_position)
        self.arm_position_idx = 0
        self.show_current_grid()
        
        return self.grid_points[idx],self.arm_start_position
        #return self.grid_points[idx],self.arm_start_position

    # ---------- STEP EXAMPLE ----------
    def step(self,a,cid=None):
        left_arm = a[0]
        right_arm = a[1]
        r = 0
        done = False
        ret_flag = False
        next_state = self.grid_points[self.arm_position_idx]
        if left_arm==0:
            if not self.object_holding():#####Build this function
                if self.is_ee_over_cube():
                    self.cid = self.pickup()
                    self.status = "Holding"
                    self.show_current_grid()
                    
                    r = 10
                    #r = 10
                    ret_flag = True
                else:
                    r=-10
                    done = True
            else:
                if self.check_drop_position():
                    r=-100
                else:
                    self.release()   # <- actually detach cube
                
                    r = 10
                    self.status = "Not Holding"
                    self.show_current_grid()
                
                    ret_flag = True
                done=True
        elif left_arm==1:
            ret_flag = True
            row = self.arm_position_idx // self.n
            col = self.arm_position_idx % self.n
            
            
            # 0 = left
            if right_arm == 0:
                col = min(col + 1, self.n-1)
            
            # 1 = right
            elif right_arm == 1:
                col = max(col - 1, 0)
            
            # 2 = down
            elif right_arm == 2:
                row = max(row - 1, 0)
            
            # 3 = up
            elif right_arm == 3:
                row = min(row + 1, self.n-1)
            
            
            new_idx = row*self.n + col
            next_state = self.grid_points[new_idx]

            self.move(next_state)
            
            self.arm_position_idx = new_idx
            
            self.show_current_grid()
        return next_state,r,done,ret_flag
# env = RobotEnv()
# cube_pos,gripper_position = env.reset()
# print("CUBE location:",cube_pos)
# print("Robot arm gripper location:",gripper_position)