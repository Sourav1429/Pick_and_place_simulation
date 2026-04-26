import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import random

from stable_baselines3 import PPO,DQN
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv


# -------------------------------------------------
# Your environment wrapped as Gym
# -------------------------------------------------

class PickPlaceEnv(gym.Env):

    metadata = {"render_modes": []}

    def __init__(self):
        super().__init__()

        self.n=3
        self.max_steps=50

        # actions:
        # 0 pick/drop
        # 1 left
        # 2 right
        # 3 down
        # 4 up
        self.action_space=spaces.Discrete(5)

        # state:
        # arm idx, cube idx, goal idx, holding flag
        self.observation_space=spaces.MultiDiscrete(
            [self.n*self.n,
             self.n*self.n,
             self.n*self.n,
             2]
        )

        self.setup_world()


    def setup_world(self):

        try:
            p.disconnect()
        except:
            pass

        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.81)

        self.TABLE_Z=0.9
        self.cid=None

        p.loadURDF(
            "table/table.urdf",
            basePosition=[1,2,-0.65],
            globalScaling=2.5
        )

        self.robot=p.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[-0.75,2,self.TABLE_Z],
            useFixedBase=True
        )

        self.ee_link=11

        self.grid=[]
        for i in range(self.n):
            for j in range(self.n):
                self.grid.append([
                    -0.5+i*0.3,
                    1.5+j*0.5,
                    self.TABLE_Z
                ])

        self.half=0.05

        self.cube = p.createMultiBody(
            baseMass=0.2,
            baseCollisionShapeIndex=
            p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[self.half]*3
            ),
            basePosition=[0,1.5,self.TABLE_Z+self.half]
        )


    def move(self,target):

        current=p.getLinkState(
            self.robot,
            self.ee_link
        )[0]

        for t in np.linspace(0,1,30):
            interp=(1-t)*np.array(current)+t*np.array(target)

            joints=p.calculateInverseKinematics(
                self.robot,
                self.ee_link,
                interp
            )

            for i in range(7):
                p.setJointMotorControl2(
                    self.robot,
                    i,
                    p.POSITION_CONTROL,
                    joints[i],
                    force=800
                )

            p.stepSimulation()


    def pickup(self):

        cube_pos,_=p.getBasePositionAndOrientation(
            self.cube
        )

        ee=p.getLinkState(
            self.robot,
            self.ee_link
        )[0]

        self.move([cube_pos[0],cube_pos[1],ee[2]])
        self.move([cube_pos[0],cube_pos[1],cube_pos[2]+0.005])

        self.cid=p.createConstraint(
            self.robot,
            self.ee_link,
            self.cube,
            -1,
            p.JOINT_FIXED,
            [0,0,0],
            [0,0,0],
            [0,0,0]
        )

        self.move([
            cube_pos[0],
            cube_pos[1],
            ee[2]+0.25
        ])


    def release(self):
        if self.cid is not None:
            p.removeConstraint(self.cid)
            self.cid=None
            for _ in range(50):
                p.stepSimulation()


    def holding(self):
        return int(self.cid is not None)


    def obs(self):
        return np.array([
            self.arm_idx,
            self.cube_idx,
            self.goal_idx,
            self.holding()
        ],dtype=np.int64)


    def reset(self,seed=None,options=None):

        super().reset(seed=seed)

        self.steps=0
        self.arm_idx=0

        self.cube_idx=random.randint(0,8)

        choices=list(range(9))
        choices.remove(self.cube_idx)
        self.goal_idx=random.choice(choices)

        c=self.grid[self.cube_idx]

        p.resetBasePositionAndOrientation(
            self.cube,
            [c[0],c[1],self.TABLE_Z+self.half],
            [0,0,0,1]
        )

        self.move(self.grid[0])

        return self.obs(),{}


    def step(self,action):

        self.steps+=1
        reward=-0.1
        terminated=False
        truncated=False

        # pick/drop
        if action==0:

            if not self.holding():

                if self.arm_idx==self.cube_idx:
                    self.pickup()
                    reward=5
                else:
                    reward=-5

            else:
                if self.arm_idx==self.goal_idx:
                    self.release()
                    reward=20
                    terminated=True
                else:
                    reward=-20
                    terminated=True


        else:
            row=self.arm_idx//self.n
            col=self.arm_idx%self.n

            if action==1:
                col=min(col+1,self.n-1)
            elif action==2:
                col=max(col-1,0)
            elif action==3:
                row=max(row-1,0)
            elif action==4:
                row=min(row+1,self.n-1)

            self.arm_idx=row*self.n+col
            self.move(self.grid[self.arm_idx])

            # shaping reward
            if self.holding():
                goal=self.goal_idx
                old_dist=abs((self.arm_idx//self.n)-(goal//self.n))+abs((self.arm_idx%self.n)-(goal%self.n))
                reward += 1.0/(1+old_dist)


        if self.steps>=self.max_steps:
            truncated=True

        return self.obs(),reward,terminated,truncated,{}


# -------------------------------------------------
# Train PPO
# -------------------------------------------------

def make_env():
    return Monitor(PickPlaceEnv())


env=DummyVecEnv([make_env])

model = DQN(
    "MlpPolicy",
    env,
    learning_rate=1e-3,
    buffer_size=100000,
    learning_starts=1000,
    batch_size=64,
    gamma=0.99,
    target_update_interval=500,
    exploration_fraction=0.4,
    exploration_final_eps=0.05,
    verbose=1
)

model.learn(200000)

model.save("dqn_pick_place_best")


# -------------------------------------------------
# Evaluate trained policy
# -------------------------------------------------

obs=env.reset()

for _ in range(100):

    action,_=model.predict(
        obs,
        deterministic=True
    )

    obs,reward,done,info=env.step(action)

    print(
        "Action:",action,
        "Reward:",reward
    )

    if done:
        break