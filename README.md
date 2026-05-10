# Human-in-the-Loop Robotic Pick-and-Place using Reinforcement Learning

A gesture-controlled robotic manipulation framework built using **PyBullet**, **MediaPipe**, **OpenCV**, and **Reinforcement Learning**.

This project combines:

- Human gesture interaction
- Robotic pick-and-place simulation
- Reinforcement learning training
- PPO-based policy optimization
- Real-time robotic control
- Interactive robot supervision

The robotic arm is controlled through webcam-based hand gestures while reinforcement learning algorithms learn optimal pick-and-place behaviors inside a simulated environment.

---

# Project Overview

The system consists of:

- A **Franka Panda robotic arm**
- A **grid-based manipulation workspace**
- A **pick-and-place object**
- A **goal container/bowl**
- A **gesture recognition interface**
- A **reinforcement learning training pipeline**

The robot learns to:

1. Navigate across the workspace
2. Pick up the object
3. Carry it safely
4. Place it inside the target container

Human gestures are used both for:
- Interactive control
- Data collection
- Human-guided reinforcement learning

---

# Features

- Franka Panda robotic arm simulation
- Gesture-controlled robotic manipulation
- Real-time webcam interaction
- Grid-based robotic workspace
- Pick-and-place environment
- PPO reinforcement learning training
- Object grasping and transport
- Randomized object placement
- Goal-conditioned manipulation
- Real-time environment rendering
- Status and grid tracking

---

# Project Structure

```bash
.
├── driver_for_Pick_place.py          # Human-controlled interaction
├── Robo_training.py                  # PPO training script
├── DEmo_wrk.py                       # Trained model demonstration
├── Pick_place_base_environment.py    # Main robotic environment
├── success.mp4                       # Successful trained rollout
├── ppo_pickplace_logs/               # PPO training logs
└── README.md
```

---

# Requirements

The project requires:

| Dependency | Purpose |
|---|---|
| Python 3 | Core programming language |
| NumPy < 2.0 | Numerical computation |
| PyBullet | Physics simulation |
| Stable-Baselines3 | Reinforcement learning |
| OpenCV | Webcam and image processing |
| MediaPipe | Hand gesture detection |
| PyTorch | Deep learning backend |

---

# Installation

## Step 1 — Create Conda Environment

Create a fresh conda environment:

```bash
conda create -n robotics_env python=3.10
conda activate robotics_env
```

---

## Step 2 — Install NumPy (< 2.0)

MediaPipe currently works best with NumPy versions below 2.0.

```bash
pip install "numpy<2.0"
```

---

## Step 3 — Install Core Dependencies

```bash
pip install pybullet
pip install stable-baselines3
pip install opencv-python
pip install mediapipe
```

---

## Step 4 — Install PyTorch

Install PyTorch based on your CUDA configuration.

Official Installation Guide:

https://pytorch.org/

Example CPU installation:

```bash
pip install torch torchvision torchaudio
```

---

## Optional — Install MuJoCo

In some systems PyBullet may conflict with physics-related packages.

If simulation issues occur, install MuJoCo:

```bash
pip install mujoco
```

---

# Running the Project

## 1. Human-Controlled Data Collection

Run:

```bash
python driver_for_Pick_place.py
```

This launches:

- PyBullet simulation
- Webcam gesture interface
- Human-controlled robotic interaction

The user can control the robotic arm entirely through gestures.

---

## 2. Train the Reinforcement Learning Model

After collecting data:

```bash
python Robo_training.py
```

Training logs are stored in:

```bash
ppo_pickplace_logs/
```

The training uses PPO from Stable-Baselines3.

---

## 3. Run the Trained Policy

To evaluate the trained robotic behavior:

```bash
python DEmo_wrk.py
```

---

# Gesture Controls

The control system uses:

- **Left Hand** → Action Mode
- **Right Hand** → Movement Direction

---

# Left Hand Controls

| Gesture | Action |
|---|---|
| ✊ Closed Fist | Pick / Drop Object |
| ✋ Open Hand | Enable Movement Mode |

---

# Right Hand Controls

| Gesture | Action |
|---|---|
| ☝️ One Finger | Move gripper one grid left |
| ✌️ Two Fingers | Move gripper one grid right |
| 🤟 Three Fingers | Move gripper one grid away from robot base |
| 🖖 Four Fingers | Move gripper one grid toward robot base |
| ✋ All Fingers Open | Close all windows and terminate episode |

---

# Gesture Logic

## Pick / Drop Mode

If the **left hand shows a closed fist**:

- The right hand input is ignored
- The robot attempts to:
  - pick the object (if within range)
  - or drop the object (if already holding)

If the object is outside grasp range:

- A large negative reward is assigned
- The episode terminates

---

## Movement Mode

If the **left hand is open**:

- The system reads the right hand gesture
- The robot moves according to the detected direction

---

# Status Display

The environment continuously displays:

- Current grid location
- Object holding status

Possible status outputs:

```text
STATUS: Not Holding Object
STATUS: Holding Object
```

The active grid position is also shown in the simulation window.

---

# Reinforcement Learning Setup

The environment supports reinforcement learning using PPO.

---

# Observation Space

The state includes:

- Robot grid position
- Object grid position
- Goal/container position
- Holding state

---

# Action Space

| Action | Meaning |
|---|---|
| 0 | Pick / Drop |
| 1 | Move Left |
| 2 | Move Right |
| 3 | Move Away from Base |
| 4 | Move Toward Base |

---

# Task Objective

The RL agent learns to:

1. Navigate to the object
2. Pick the object
3. Transport the object safely
4. Place the object inside the target bowl/container

---

# Simulation Environment

| Component | Description |
|---|---|
| Physics Engine | PyBullet |
| Robot | Franka Panda Arm |
| Workspace | Grid-based table |
| Goal | Bowl/container |
| Vision | OpenCV + MediaPipe |
| RL Framework | Stable-Baselines3 |

---

# Expected Output

After successful training, the learned robotic behavior should resemble the rollout shown in:

```bash
success.mp4
```

The robot should:

- Move smoothly across the workspace
- Pick the object correctly
- Transport it safely
- Drop it inside the goal container

---

# Future Improvements

Possible future extensions include:

- Sim-to-real transfer
- Multi-object manipulation
- Multi-agent robotics
- Safe reinforcement learning
- Offline reinforcement learning
- Vision-language robotic control
- Human preference learning
- Real-world robotic deployment

---

# Acknowledgements

Built using:

- PyBullet
- OpenCV
- MediaPipe
- Stable-Baselines3
- PyTorch

This project combines concepts from:

- Reinforcement Learning
- Human-in-the-Loop Robotics
- Gesture Recognition
- Robotic Manipulation
- Safe Reinforcement Learning
