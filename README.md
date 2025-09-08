# Robot Control System

A state machine based robot control system for pick and place tasks using robosuite simulation.

## Overview

This system controls a robotic arm to perform pick and place operations through a series of predefined states. The robot moves through states like moving to object, opening gripper, descending, grasping, lifting, moving to goal, and releasing the object.

## Installation

```bash
pip install robosuite numpy opencv-python
```

## Usage

### Basic Usage

```bash
python pickplace.py
```

### Command Line Parameters

```bash
python pickplace.py [options]
```

#### Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `--episodes` | int | 3 | Number of episodes to run |
| `--max-steps` | int | 1000 | Maximum steps per episode |
| `--position-tolerance` | float | 0.02 | Position tolerance for state transitions (meters) |
| `--gripper-tolerance` | float | 0.005 | Gripper tolerance for state transitions (meters) |
| `--state-timeout` | int | 60 | Timeout for each state (steps) |
| `--grasp-height` | float | -0.005 | Height below object for grasping (meters) |
| `--lift-height` | float | 0.2 | Height to lift object (meters) |
| `--place-height` | float | 0.05 | Height above goal for placing (meters) |
| `--goal-x` | float | 0.2 | Goal position X coordinate |
| `--goal-y` | float | 0.42 | Goal position Y coordinate |
| `--goal-z` | float | 0.86 | Goal position Z coordinate |
| `--no-video` | flag | False | Disable video recording |
| `--video-fps` | int | 30 | Video recording frame rate |
| `--print-freq` | int | 20 | Print frequency (steps) |

#### Examples

**Run with custom parameters:**
```bash
python pickplace.py --episodes 5 --position-tolerance 0.01 --grasp-height -0.01
```

**Run without video recording:**
```bash
python pickplace.py --no-video --episodes 10
```

**Run with custom goal position:**
```bash
python pickplace.py --goal-x 0.3 --goal-y 0.5 --goal-z 0.9
```

**Run with high precision:**
```bash
python pickplace.py --position-tolerance 0.01 --gripper-tolerance 0.002 --state-timeout 100
```

## State Machine

The robot follows this sequence of states:

1. **IDLE** → **MOVE_TO_OBJECT**: Move to position above object
2. **MOVE_TO_OBJECT** → **OPEN_GRIPPER**: Open gripper to maximum width
3. **OPEN_GRIPPER** → **DESCEND_TO_OBJECT**: Descend to object
4. **DESCEND_TO_OBJECT** → **GRASP_OBJECT**: Close gripper to grasp
5. **GRASP_OBJECT** → **LIFT_OBJECT**: Lift object to safe height
6. **LIFT_OBJECT** → **MOVE_TO_GOAL**: Move horizontally to goal position
7. **MOVE_TO_GOAL** → **DESCEND_TO_GOAL**: Descend to goal position
8. **DESCEND_TO_GOAL** → **RELEASE_OBJECT**: Open gripper to release
9. **RELEASE_OBJECT** → **LIFT_AWAY**: Lift arm away from goal
10. **LIFT_AWAY** → **COMPLETE**: Task completed

## Parameter Tuning Guide

### Position Tolerance (`--position-tolerance`)
- **Lower values (0.01-0.02)**: More precise positioning, slower execution
- **Higher values (0.05-0.1)**: Faster execution, less precise
- **Recommended**: 0.02 for balanced performance

### Gripper Tolerance (`--gripper-tolerance`)
- **Lower values (0.002-0.005)**: More precise gripper control
- **Higher values (0.01-0.02)**: Faster gripper operations
- **Recommended**: 0.005 for reliable grasping

### State Timeout (`--state-timeout`)
- **Lower values (30-60)**: Faster execution, may cause timeouts
- **Higher values (100-200)**: More reliable, slower execution
- **Recommended**: 60 for most scenarios

### Grasp Height (`--grasp-height`)
- **Negative values (-0.01 to -0.005)**: Grasp below object surface
- **Zero or positive**: May not grasp properly
- **Recommended**: -0.005 for reliable grasping

### Goal Position
- **X coordinate**: Front/back position (0.1 to 0.4)
- **Y coordinate**: Left/right position (0.1 to 0.5)
- **Z coordinate**: Height (0.8 to 1.0)
- **Recommended**: (0.2, 0.42, 0.86) for good visibility

## Output

### Console Output
- State transitions and progress updates
- Performance metrics (success rate, steps, collisions)
- Error messages and warnings

### Video Output
- Saved to `videos/` directory
- Filename format: `pickplace_ep{episode_id}_{success/fail}.mp4`
- High quality (1280x720, 30fps)
- Proper orientation (not upside-down)

### Performance Metrics
- **Task Success Rate**: Percentage of successful pick and place operations
- **Completion Rate**: Percentage of episodes that completed the full sequence
- **Average Steps**: Mean number of steps per episode
- **Total Collisions**: Number of collision events

## File Structure

```
.
├── pickplace.py    # Main control script
├── README.md              # This documentation
├── videos/                # Video output directory
│   ├── pickplace_ep0_success.mp4
│   ├── pickplace_ep1_success.mp4
│   └── pickplace_ep2_success.mp4
└── robosuite_demo.py      # Reference implementation
└── resultsrecord.txt
```

## Dependencies

- Python 3.7+
- robosuite
- numpy
- opencv-python
- argparse (built-in)

## License

This project is for educational purposes.
