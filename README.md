Requirements 
1) Pybullet
2) stable baselines
3) opencv
4) mediapipe
To install these requirements

Create an environment in conda with python3 and numpy version<2.0 and activate it
pip install pybullet
pip install stable_baselines3
pip install opencv
pip install mediapipe

The environments are mostly working but if incase any trouble faced with pybullet, check your mujoco (if dowloaded)
else download mujoco using
pip install mujoco

The code needs pytorch so install pytorch if not already installed.

How to run the file?

Feeding in data while training: python driver_for_Pick_place.py

The instructions are as follow
Left arm actions: a) fist closed b) fist open
Right arm actions a) showing one finger ------> Take the end effector (gripper) one grid step left from te robot's POV
b) showing two fingers -----------------------> Take the gripper one step right from robot's POV
c) showing three fingers ---------------------> Take gripper one step away fro robot's base
d) showing four fingers ----------------------> Take gripper one step towards's the base.
e) Showing all fingers -----------------------> If left fist is open then this mean close all window and episode ends.

GESTURE CONSTROLS

1) If left arm shows fist closed -----> Ignore right arm reading and pick the object if the object is in range other wise provide a high negative reward and end the episode.
Also this action would be valid if the object is held. 

Before picking the object the STATUS bar displays "Not Holding the object" and after the object is picked the status changes to "Holding the object". The STATUS BAR also shows the Grid position.

2) If the left fist is open then the camera considers the right hand's input and takes action as listed above.

TRAINING THE MODEL:
Run the following file "python Robo_training.py" after capturing some data. For now the data is stored in ppo_pickplace_logs and will use it only.

Testing it:
Run python DEmo_wrk.py

After training it should look like the video in "success.mp4"
