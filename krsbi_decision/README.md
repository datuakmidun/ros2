# krsbi_decision

The high-level strategy and decision-making logic for KRSBI-B Soccer Robot.
Built using **Behavior Trees** (`py_trees`) and a ROS 2 Node architecture.

## 📋 Overview

Components:

- **Game Controller**: Manages game states (INITIAL, READY, SET, PLAYING) via referee commands.
- **World Model**: Maintains a unified state of the game (Robot Pose, Ball Position, etc.).
- **Strategy Manager**: Executes behavior trees based on the assigned role (Striker, Goalie).
- **Behavior Tree**: Modular actions (GoTo, Kick) and conditions (IsBallVisible).

## 📁 Package Structure

```
krsbi_decision/
├── krsbi_decision/
│   ├── game_controller.py       # Referee Interface
│   ├── strategy_manager.py      # Main BT Executor
│   ├── world_model/
│   │   └── world_state.py       # Data structures
│   └── behavior_tree/
│       ├── actions.py           # Leaf nodes (Actuators)
│       ├── conditions.py        # Leaf nodes (Sensors)
│       └── roles.py             # Tree definitions (Striker, Goalie)
├── config/
│   └── strategy_params.yaml     # Game constants
├── launch/
│   └── decision_bringup.launch.py
```

## 🚀 Usage

### Launch Decision System

```bash
# Build
colcon build --packages-select krsbi_decision
source install/setup.bash

# Run Striker
ros2 launch krsbi_decision decision_bringup.launch.py role:=striker

# Run Goalkeeper
ros2 launch krsbi_decision decision_bringup.launch.py role:=goalie
```

### Referee Simulation

```bash
# Start Game
ros2 topic pub /referee std_msgs/String "data: START" --once

# Stop Game
ros2 topic pub /referee std_msgs/String "data: STOP" --once
```

## 🧠 Behavior Logic

### Striker

1.  **Check Game State**: Must be `PLAYING`.
2.  **Kick Sequence**: If ball is visible and in range -> Align -> Kick.
3.  **Approach Sequence**: If ball is visible -> Follow Ball.
4.  **Search**: If ball lost -> Spin/Wait.

### Goalkeeper

1.  **Clear Ball**: If ball is dangerously close -> Kick.
2.  **Defend**: Maintain position at goal line.

## 🔗 Interfaces

- **Subscribes**:
  - `/odom`: Robot localization.
  - `/krsbi/vision/ball`: Ball detection.
  - `/referee`: Game commands.
- **Publishes**:
  - `/cmd_vel`: Movement commands (fallback/direct).
  - `/krsbi/behavior/command`: High-level behavior requests (e.g. `FOLLOW_BALL`).
  - `/krsbi/decision/state`: Current behavior status.
