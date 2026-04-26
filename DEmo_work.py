# -*- coding: utf-8 -*-
"""
Created on Sat Apr 25 19:32:39 2026

@author: gangu
"""

from stable_baselines3 import DQN
import time
from Pick_place_base_environment import RobotEnv

# load trained model
#model = DQN.load("RHC_UCRL_pick_place_best.zip")


# GUI environment
#env = RobotEnv()
#cube_pos,obs,goal_idx = env.reset()
#obs, _ = env.reset()

# done = False
# truncated = False

# while not (done or truncated):

#     action, _ = model.predict(
#         obs,
#         deterministic=True
#     )
#     action_map={'Left':[0,1,1,1,1],'Right':[1,1,2,3,4]}
#     left = action_map['Left'][0] if action==0 else 1
#     right = action_map['Right'][action]

#     next_state,r,done,ret_flag = env.step([left,right])

#     print(
#         "Action:", action,
#         "Reward:", r
#     )

#     time.sleep(1.0)g


from stable_baselines3 import DQN
import time

from Pick_place_base_environment import RobotEnv


# ----------------------------
# Load trained model
# ----------------------------

model = DQN.load("RHC_UCRL_pick_place_best.zip")


# ----------------------------
# Real PyBullet GUI environment
# ----------------------------

env = RobotEnv()

cube_pos, arm_pos, goal_idx = env.reset()

print("Cube grid:", env.cube_idx)
print("Goal grid:", goal_idx)


# ----------------------------
# Action translator
# RL action -> gesture action
# ----------------------------

def decode_action(a):

    if a==0:
        return [0,0]   # pick/drop

    elif a==1:
        return [1,0]   # move left

    elif a==2:
        return [1,1]   # move right

    elif a==3:
        return [1,2]   # move down

    elif a==4:
        return [1,3]   # move up



# ----------------------------
# State for policy
# ----------------------------

obs = [
    env.arm_position_idx,
    env.cube_idx,
    env.goal_idx,
    int(env.object_holding())
]


done=False


# ----------------------------
# Roll out learned policy
# ----------------------------

while not done:

    action,_ = model.predict(
        obs,
        deterministic=True
    )

    print("RL action:", action)

    gesture_action = decode_action(
        int(action)
    )

    print("Gesture action:", gesture_action)


    next_state,reward,done,_ = env.step(
        gesture_action
    )

    print(
        "Reward:",
        reward,
        "Done:",
        done
    )


    # update policy observation
    obs = [
        env.arm_position_idx,
        env.cube_idx,
        env.goal_idx,
        int(env.object_holding())
    ]

    time.sleep(1)


print("Episode finished.")