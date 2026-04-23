# -*- coding: utf-8 -*-
"""
Created on Thu Apr 23 15:35:01 2026

@author: gangu
"""
from fist_closed_action_checking import GestureController
from Pick_place_base_environment import RobotEnv
gc = GestureController()
env = RobotEnv()
need_user_input = 1
done = False
cube_pos,gripper_position = env.reset()
while not done:
    gc.update_camera()   # keeps feed alive

    # robot logic...
    
    if need_user_input:
        left,right = gc.get_gesture()
        print("Left:",left)
        print("Right:",right)
        if(left==1 and right==5):
            gc.close()
            break
        elif left==None or (left!=None and right==None):
            continue
        else:
            right-=1
            action = [left,right]
            next_state,r,done,ret_flag = env.step(action)
            print("reward received:",r)
            if done:
                input("Episode complete. Press any key to continue")
                gc.close()
            

        # control returns immediately here
        #do_action(left,right)