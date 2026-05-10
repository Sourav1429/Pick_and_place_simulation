# Human-in-the-Loop Robotic Pick-and-Place using Reinforcement Learning

A gesture-controlled robotic manipulation environment built using PyBullet, MediaPipe, OpenCV, and Reinforcement Learning.  
The project combines:

- Human gesture interaction
- Robotic pick-and-place simulation
- Reinforcement learning training
- PPO-based policy optimization
- Real-time robotic control

The robotic arm is controlled using hand gestures captured through a webcam, while reinforcement learning algorithms learn optimal pick-and-place behaviors in the environment.

---

# Features

- Franka Panda robotic arm simulation using PyBullet
- Human gesture-controlled interaction
- Real-time webcam input using OpenCV + MediaPipe
- Grid-based robotic manipulation environment
- Pick-and-place task learning
- PPO reinforcement learning training
- Real-time environment rendering
- Status display for object holding and grid position
- Randomized object placement for training diversity

---

# Project Structure

```bash
.
├── driver_for_Pick_place.py      # Human-controlled data collection
├── Robo_training.py              # RL training script
├── DEmo_wrk.py                   # Demo/testing script
├── Pick_place_base_environment.py
├── success.mp4                   # Example successful rollout
├── ppo_pickplace_logs/           # Training logs
└── README.md
