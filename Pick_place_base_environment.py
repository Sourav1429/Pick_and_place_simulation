import pybullet as p
import pybullet_data
import time
import numpy as np

class Pick_Place_Env:
    def __init__(self, gui=True):
        self.gui = gui
        self.client = None

        # workspace params
        self.TABLE_CENTER = [1, 2, -0.65]
        self.TABLE_Z = 0.9
        self.ee_link = 11

        self._connect()
        self._setup_world()
        self._create_grid()
        self._spawn_objects()

    # ------------------ CONNECTION ------------------
    def _connect(self):
        try:
            p.disconnect()
        except:
            pass

        if self.gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # camera
        if self.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=3.5,
                cameraYaw=0,
                cameraPitch=-30,
                cameraTargetPosition=[0.55, -0.15, 0.2]
            )

    # ------------------ WORLD ------------------
    def _setup_world(self):
        # table
        self.table = p.loadURDF(
            "table/table.urdf",
            basePosition=self.TABLE_CENTER,
            globalScaling=2.5
        )

        # robot
        self.robot = p.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[-0.75, 2.0, self.TABLE_Z],
            useFixedBase=True
        )

    # ------------------ GRID ------------------
    def _create_grid(self):
        self.grid_points = []

        for i in range(3):
            for j in range(3):
                x = 0 + i * 1
                y = 0.9 + j * 0.8
                z = self.TABLE_Z
                self.grid_points.append([x, y, z])

        # visualize
        for idx, pt in enumerate(self.grid_points):
            p.addUserDebugText(
                str(idx),
                [pt[0], pt[1], pt[2] + 0.02],
                textColorRGB=[1, 0, 0],
                textSize=1.5
            )

    # ------------------ OBJECTS ------------------
    def _spawn_objects(self, start_idx=0):
        cube_size = 0.1
        half = cube_size / 2

        self.cube = p.createMultiBody(
            baseMass=0.2,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[half]*3
            ),
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_BOX, halfExtents=[half]*3,
                rgbaColor=[0.2, 0.6, 0.9, 1]
            ),
            basePosition=[
                self.grid_points[start_idx][0],
                self.grid_points[start_idx][1],
                self.TABLE_Z + half
            ]
        )

env = Pick_Place_Env(gui=True)