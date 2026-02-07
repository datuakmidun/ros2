# Changelog

All notable changes to the `krsbi_decision` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-07

### Added

#### Strategy

- **strategy_manager.py**: Main ROS 2 node executing behavior trees.
- Supports role switching (Striker, Goalie) via ROS parameters.
- Maintains `WorldModel` from sensor data.

#### Behavior Trees

- **py_trees integration**: Modular behavior tree implementation.
- **roles.py**: Defines Striker (Attack) and Goalie (Defend) trees.
- **actions.py**: Implements GoToPosition, FollowBall, KickBall.
- **conditions.py**: Implements IsBallVisible, IsBallInKickRange, IsGameState.

#### Game Control

- **game_controller.py**: Interfaces with referee commands (START, STOP, GOAL).
- Publishes `/krsbi/game/state` for system-wide synchronization.

#### World Model

- **world_state.py**: Dataclasses for RobotPose and BallState.
- Tracks game phase and scores.

#### Configuration

- **strategy_params.yaml**: Defines game rules and role parameters.
- **decision_bringup.launch.py**: Launches game controller and strategy manager.

### Notes

- Requires `krsbi_control` for movement execution.
- Relies on `krsbi_vision` for perception input.
