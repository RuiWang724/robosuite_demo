#!/usr/bin/env python3
"""
Robot Control System
State machine based robot control for pick and place tasks
"""

import os
import warnings
import numpy as np
import cv2
import robosuite as suite
from robosuite.controllers import load_composite_controller_config
import logging
import argparse

# Suppress warning messages
warnings.filterwarnings("ignore")
os.environ['PYTHONWARNINGS'] = 'ignore'
logging.getLogger().setLevel(logging.ERROR)


class RobotController:
    """Robot controller based on state machine"""
    
    def __init__(self, config=None):
        # State definition
        self.current_state = "IDLE"
        self.state_history = []
        
        # Position information
        self.obj_pos = None
        self.goal_pos = None
        
        # Configuration
        self.config = config or {}
        
    def reset(self):
        """Reset controller state"""
        self.current_state = "IDLE"
        self.state_history = []
        self.obj_pos = None
        self.goal_pos = None
        
        # Control parameters
        self.POSITION_TOLERANCE = self.config.get('position_tolerance', 0.02)  # Position tolerance (2cm) - ensure precise positioning above object before vertical descent
        self.GRIPPER_TOLERANCE = self.config.get('gripper_tolerance', 0.005)  # Gripper tolerance (0.5cm)
        self.STATE_TIMEOUT = self.config.get('state_timeout', 60)  # Increased timeout to ensure enough time to complete each state
        
        # Height parameters
        self.HOVER_HEIGHT = 0.10
        self.GRASP_HEIGHT = self.config.get('grasp_height', -0.005)  # Descend to 0.5cm below object to avoid collision
        self.PLACE_HEIGHT = self.config.get('place_height', 0.05)
        self.LIFT_HEIGHT = self.config.get('lift_height', 0.2)  # Reduced lift height from 0.20m to 0.10m
        
    def update_positions_from_obs(self, obs):
        """Update object and goal positions from observations"""
        if "Can_pos" in obs:
            self.obj_pos = obs["Can_pos"][:3]
        
        if self.goal_pos is None:
            # Use configured goal position or default
            goal_x = self.config.get('goal_x', 0.2)
            goal_y = self.config.get('goal_y', 0.42)
            goal_z = self.config.get('goal_z', 0.86)
            self.goal_pos = np.array([goal_x, goal_y, goal_z])
    
    def get_instruction(self, obs, step):
        """Generate robot control instruction based on current state"""
        eef_pos = obs["robot0_eef_pos"]
        gripper_pos = obs["robot0_gripper_qpos"]
        
        # Update position information
        self.update_positions_from_obs(obs)
        
        # Generate instruction based on current state and observations
        if self.current_state == "IDLE":
            instruction = f"Move robot arm to position above object at {self.obj_pos} with height {self.HOVER_HEIGHT}"
            target_pos = np.array([self.obj_pos[0], self.obj_pos[1], self.obj_pos[2] + self.HOVER_HEIGHT])
            target_gripper = None
            
        elif self.current_state == "MOVE_TO_OBJECT":
            instruction = f"Keep moving to object position, current error: {np.linalg.norm(eef_pos - np.array([self.obj_pos[0], self.obj_pos[1], self.obj_pos[2] + self.HOVER_HEIGHT])):.3f}"
            target_pos = np.array([self.obj_pos[0], self.obj_pos[1], self.obj_pos[2] + self.HOVER_HEIGHT])
            target_gripper = None
            
        elif self.current_state == "OPEN_GRIPPER":
            instruction = "Open gripper to maximum width for grasping"
            target_pos = None
            target_gripper = 0.039  # Maximum gripper open position
            
        elif self.current_state == "DESCEND_TO_OBJECT":
            instruction = f"Descend to object at height {self.GRASP_HEIGHT} above object"
            target_pos = np.array([self.obj_pos[0], self.obj_pos[1], self.obj_pos[2] + self.GRASP_HEIGHT])
            target_gripper = None
            
        elif self.current_state == "GRASP_OBJECT":
            instruction = "Close gripper to grasp the object"
            target_pos = None
            target_gripper = 0.001  # Gripper closed position (close to 0)
            
        elif self.current_state == "LIFT_OBJECT":
            instruction = f"Lift object to height {self.LIFT_HEIGHT} above current position"
            target_pos = np.array([self.obj_pos[0], self.obj_pos[1], self.obj_pos[2] + self.LIFT_HEIGHT])
            target_gripper = None
            
        elif self.current_state == "MOVE_TO_GOAL":
            instruction = f"Move to goal position above {self.goal_pos} with height {self.LIFT_HEIGHT}"
            target_pos = np.array([self.goal_pos[0], self.goal_pos[1], self.goal_pos[2] + self.LIFT_HEIGHT])
            target_gripper = None
            
        elif self.current_state == "DESCEND_TO_GOAL":
            instruction = f"Descend to goal position at height {self.PLACE_HEIGHT} above goal"
            target_pos = np.array([self.goal_pos[0], self.goal_pos[1], self.goal_pos[2] + self.PLACE_HEIGHT])
            target_gripper = None
            
        elif self.current_state == "RELEASE_OBJECT":
            instruction = "Open gripper to release the object"
            target_pos = None
            target_gripper = 0.039  # Maximum gripper open position
            
        elif self.current_state == "LIFT_AWAY":
            instruction = f"Lift arm away to height {self.LIFT_HEIGHT} above goal"
            target_pos = np.array([self.goal_pos[0], self.goal_pos[1], self.goal_pos[2] + self.LIFT_HEIGHT])
            target_gripper = None
            
        else:
            instruction = "Task completed"
            target_pos = None
            target_gripper = None
        
        return instruction, target_pos, target_gripper
    
    def execute_instruction(self, obs, step):
        """Execute robot control instruction"""
        eef_pos = obs["robot0_eef_pos"]
        gripper_pos = obs["robot0_gripper_qpos"]
        
        # Get instruction
        instruction, target_pos, target_gripper = self.get_instruction(obs, step)
        
        # Print instruction and status information (configurable frequency)
        print_freq = self.config.get('print_freq', 20)
        if step % print_freq == 0:
            print(f"Instruction: {instruction}")
            if target_pos is not None:
                distance = np.linalg.norm(eef_pos - target_pos)
                print(f"Current pos: {eef_pos}, Target pos: {target_pos}, Distance: {distance:.3f}")
            if target_gripper is not None:
                print(f"Current gripper: {gripper_pos[0]:.3f}, Target gripper: {target_gripper:.3f}")
        
        # State machine logic
        self.update_state_machine(eef_pos, gripper_pos, step)
        
        # Calculate action
        if target_pos is not None:
            pos_diff = target_pos - eef_pos
            # Add collision avoidance: reduce speed when too close
            distance = np.linalg.norm(pos_diff)
            if distance < 0.05:  # When very close
                pos_diff = pos_diff * 0.5  # Reduce speed
        else:
            pos_diff = np.array([0.0, 0.0, 0.0])
        
        if target_gripper is not None:
            # Gripper position is 2D array, we use the first element
            # Note: positive action value closes gripper, negative opens gripper
            grip_diff = gripper_pos[0] - target_gripper  # Reverse control direction
        else:
            grip_diff = 0.0
        
        action = np.concatenate([
            pos_diff * 5.0,          # Position control - significantly increase speed
            [0.0, 0.0, 0.0],         # Orientation control
            [grip_diff * 25.0]       # Gripper control - increase gripper control strength
        ])
        
        return np.clip(action, -0.8, 0.8)  # Use standard action range
    
    def update_state_machine(self, eef_pos, gripper_pos, step):
        """Update state machine"""
        # Check timeout
        if not hasattr(self, 'state_start_step'):
            self.state_start_step = step
        elif step - self.state_start_step > self.STATE_TIMEOUT:
            print(f"State {self.current_state} timeout, forcing transition")
            self.state_start_step = step
            # Force state transition
            if self.current_state == "GRASP_OBJECT":
                self.current_state = "LIFT_OBJECT"
                print(f"Force transition: GRASP_OBJECT -> LIFT_OBJECT")
                return
            elif self.current_state == "OPEN_GRIPPER":
                self.current_state = "DESCEND_TO_OBJECT"
                print(f"Force transition: OPEN_GRIPPER -> DESCEND_TO_OBJECT")
                return
            elif self.current_state == "LIFT_OBJECT":
                self.current_state = "MOVE_TO_GOAL"
                print(f"Force transition: LIFT_OBJECT -> MOVE_TO_GOAL")
                return
            elif self.current_state == "MOVE_TO_GOAL":
                self.current_state = "DESCEND_TO_GOAL"
                print(f"Force transition: MOVE_TO_GOAL -> DESCEND_TO_GOAL")
                return
            elif self.current_state == "DESCEND_TO_GOAL":
                self.current_state = "RELEASE_OBJECT"
                print(f"Force transition: DESCEND_TO_GOAL -> RELEASE_OBJECT")
                return
            elif self.current_state == "RELEASE_OBJECT":
                self.current_state = "LIFT_AWAY"
                print(f"Force transition: RELEASE_OBJECT -> LIFT_AWAY")
                return
        
        # Check if position is reached
        if self.current_state in ["MOVE_TO_OBJECT", "DESCEND_TO_OBJECT", "LIFT_OBJECT", 
                                 "MOVE_TO_GOAL", "DESCEND_TO_GOAL", "LIFT_AWAY"]:
            if self.current_state == "MOVE_TO_OBJECT":
                target_pos = np.array([self.obj_pos[0], self.obj_pos[1], self.obj_pos[2] + self.HOVER_HEIGHT])
            elif self.current_state == "DESCEND_TO_OBJECT":
                target_pos = np.array([self.obj_pos[0], self.obj_pos[1], self.obj_pos[2] + self.GRASP_HEIGHT])
            elif self.current_state == "LIFT_OBJECT":
                # Keep current X, Y position, only lift vertically to fixed safe height
                target_pos = np.array([eef_pos[0], eef_pos[1], 1.0])  # Lift to 1.0m safe height
            elif self.current_state == "MOVE_TO_GOAL":
                # Keep fixed safe height, only move horizontally to target position above
                target_pos = np.array([self.goal_pos[0], self.goal_pos[1], 1.0])  # Keep 1.0m safe height
            elif self.current_state == "DESCEND_TO_GOAL":
                target_pos = np.array([self.goal_pos[0], self.goal_pos[1], self.goal_pos[2] + self.PLACE_HEIGHT])
            elif self.current_state == "LIFT_AWAY":
                target_pos = np.array([self.goal_pos[0], self.goal_pos[1], self.goal_pos[2] + self.LIFT_HEIGHT])
            
            position_reached = np.linalg.norm(eef_pos - target_pos) < self.POSITION_TOLERANCE
        else:
            position_reached = True
        
        # Check if gripper is reached
        if self.current_state in ["OPEN_GRIPPER", "GRASP_OBJECT", "RELEASE_OBJECT"]:
            if self.current_state == "OPEN_GRIPPER":
                target_gripper = 0.039  # Maximum gripper open position
            elif self.current_state == "GRASP_OBJECT":
                target_gripper = 0.001  # Gripper closed position (close to 0)
            elif self.current_state == "RELEASE_OBJECT":
                target_gripper = 0.039  # Maximum gripper open position
            
            # Gripper position is 2D array, we use the first element
            gripper_reached = abs(gripper_pos[0] - target_gripper) < self.GRIPPER_TOLERANCE
        else:
            gripper_reached = True
        
        # State transitions
        if self.current_state == "IDLE":
            self.current_state = "MOVE_TO_OBJECT"
            self.state_start_step = step
            print(f"State transition: IDLE -> MOVE_TO_OBJECT")
            
        elif self.current_state == "MOVE_TO_OBJECT" and position_reached:
            self.current_state = "OPEN_GRIPPER"
            self.state_start_step = step
            print(f"State transition: MOVE_TO_OBJECT -> OPEN_GRIPPER")
            
        elif self.current_state == "OPEN_GRIPPER" and gripper_reached:
            self.current_state = "DESCEND_TO_OBJECT"
            self.state_start_step = step
            print(f"State transition: OPEN_GRIPPER -> DESCEND_TO_OBJECT")
            
        elif self.current_state == "DESCEND_TO_OBJECT" and position_reached:
            self.current_state = "GRASP_OBJECT"
            self.state_start_step = step
            print(f"State transition: DESCEND_TO_OBJECT -> GRASP_OBJECT")
            
        elif self.current_state == "GRASP_OBJECT" and gripper_reached:
            self.current_state = "LIFT_OBJECT"
            self.state_start_step = step
            print(f"State transition: GRASP_OBJECT -> LIFT_OBJECT")
            
        elif self.current_state == "LIFT_OBJECT" and position_reached:
            self.current_state = "MOVE_TO_GOAL"
            self.state_start_step = step
            print(f"State transition: LIFT_OBJECT -> MOVE_TO_GOAL")
            
        elif self.current_state == "MOVE_TO_GOAL" and position_reached:
            self.current_state = "DESCEND_TO_GOAL"
            self.state_start_step = step
            print(f"State transition: MOVE_TO_GOAL -> DESCEND_TO_GOAL")
            
        elif self.current_state == "DESCEND_TO_GOAL" and position_reached:
            self.current_state = "RELEASE_OBJECT"
            self.state_start_step = step
            print(f"State transition: DESCEND_TO_GOAL -> RELEASE_OBJECT")
            
        elif self.current_state == "RELEASE_OBJECT" and gripper_reached:
            self.current_state = "LIFT_AWAY"
            self.state_start_step = step
            print(f"State transition: RELEASE_OBJECT -> LIFT_AWAY")
            
        elif self.current_state == "LIFT_AWAY" and position_reached:
            self.current_state = "COMPLETE"
            self.state_start_step = step
            print(f"State transition: LIFT_AWAY -> COMPLETE")
    
    def is_complete(self):
        """Check if task is complete"""
        return self.current_state == "COMPLETE"


def pick_place_policy(obs, step, controller):
    """Pick and place policy function"""
    return controller.execute_instruction(obs, step)


def run_episode(env, episode_id=0, save_video=True, config=None):
    """Run robot control episode"""
    obs = env.reset()
    max_steps = config.get('max_steps', 1000) if config else 1000  # Increased total steps to ensure enough time to complete full pick-and-place process
    collision_count = 0
    video_frames = []
    
    # Create controller and reset
    controller = RobotController(config)
    controller.reset()
    
    for step in range(max_steps):
        action = pick_place_policy(obs, step, controller)
        obs, reward, done, info = env.step(action)
        
        # Collision detection
        if env.sim.data.ncon > 0:
            collision_count += 1
        
        # Record video frames
        if save_video and "frontview_image" in obs:
            frame = obs["frontview_image"]
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # Flip image to solve upside-down problem
            frame = cv2.flip(frame, 0)  # Vertical flip
            video_frames.append(frame)
        
        # Check if task is complete
        if controller.is_complete():
            print(f"Task completed at step {step}")
            break
    
    # Determine success
    success = env._check_success()
    
    # Save video
    if save_video and video_frames:
        os.makedirs("videos", exist_ok=True)
        status = "success" if success else "fail"
        path = f"videos/pickplace_ep{episode_id}_{status}.mp4"
        height, width, _ = video_frames[0].shape
        # Use higher quality encoder and configurable frame rate for better clarity
        fps = config.get('video_fps', 30) if config else 30
        out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"XVID"), fps, (width, height))
        for frame in video_frames:
            out.write(frame)
        out.release()
        print(f"Video saved: {path}")
    
    return {
        "success": success,
        "steps": step + 1,
        "collisions": collision_count,
        "complete": controller.is_complete()
    }


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Robot Control System for Pick and Place Tasks')
    
    # Episode parameters
    parser.add_argument('--episodes', type=int, default=3, help='Number of episodes to run (default: 3)')
    parser.add_argument('--max-steps', type=int, default=1000, help='Maximum steps per episode (default: 1000)')
    
    # Control parameters
    parser.add_argument('--position-tolerance', type=float, default=0.02, help='Position tolerance for state transitions in meters (default: 0.02)')
    parser.add_argument('--gripper-tolerance', type=float, default=0.005, help='Gripper tolerance for state transitions in meters (default: 0.005)')
    parser.add_argument('--state-timeout', type=int, default=60, help='Timeout for each state in steps (default: 60)')
    
    # Height parameters
    parser.add_argument('--grasp-height', type=float, default=-0.005, help='Height below object for grasping in meters (default: -0.005)')
    parser.add_argument('--lift-height', type=float, default=0.2, help='Height to lift object in meters (default: 0.2)')
    parser.add_argument('--place-height', type=float, default=0.05, help='Height above goal for placing in meters (default: 0.05)')
    
    # Goal position parameters
    parser.add_argument('--goal-x', type=float, default=0.2, help='Goal position X coordinate (default: 0.2)')
    parser.add_argument('--goal-y', type=float, default=0.42, help='Goal position Y coordinate (default: 0.42)')
    parser.add_argument('--goal-z', type=float, default=0.86, help='Goal position Z coordinate (default: 0.86)')
    
    # Video parameters
    parser.add_argument('--no-video', action='store_true', help='Disable video recording')
    parser.add_argument('--video-fps', type=int, default=30, help='Video recording frame rate (default: 30)')
    
    # Output parameters
    parser.add_argument('--print-freq', type=int, default=20, help='Print frequency in steps (default: 20)')
    
    return parser.parse_args()


def main():
    """Main function"""
    args = parse_arguments()
    
    print("Starting robot control system")
    print(f"Configuration: {args.episodes} episodes, {args.max_steps} max steps")
    print(f"Position tolerance: {args.position_tolerance}, Gripper tolerance: {args.gripper_tolerance}")
    print(f"Goal position: ({args.goal_x}, {args.goal_y}, {args.goal_z})")
    print(f"Video recording: {'Disabled' if args.no_video else f'Enabled ({args.video_fps} FPS)'}")
    
    # Create configuration dictionary
    config = {
        'max_steps': args.max_steps,
        'position_tolerance': args.position_tolerance,
        'gripper_tolerance': args.gripper_tolerance,
        'state_timeout': args.state_timeout,
        'grasp_height': args.grasp_height,
        'lift_height': args.lift_height,
        'place_height': args.place_height,
        'goal_x': args.goal_x,
        'goal_y': args.goal_y,
        'goal_z': args.goal_z,
        'video_fps': args.video_fps,
        'print_freq': args.print_freq
    }
    
    # Create environment
    controller_config = load_composite_controller_config(controller="BASIC", robot="Panda")
    
    env = suite.make(
        env_name="PickPlaceCan",
        robots="Panda",
        controller_configs=controller_config,
        use_camera_obs=True,
        has_offscreen_renderer=True,
        has_renderer=False,
        camera_names="frontview",
        camera_heights=720,
        camera_widths=1280,
        reward_shaping=True,
        initialization_noise=None
    )
    
    num_episodes = args.episodes
    results = []
    
    for ep in range(num_episodes):
        print(f"\nEpisode {ep + 1}/{num_episodes}")
        result = run_episode(env, episode_id=ep, save_video=not args.no_video, config=config)
        print(f"Success: {result['success']} | Steps: {result['steps']} | Collisions: {result['collisions']} | Complete: {result['complete']}")
        results.append(result)
    
    env.close()
    
    # Summary statistics
    total_success = sum(r["success"] for r in results)
    total_complete = sum(r["complete"] for r in results)
    avg_steps = np.mean([r["steps"] for r in results])
    total_collisions = sum(r["collisions"] for r in results)
    
    print("\nEvaluation Summary")
    print("=" * 50)
    print(f"Task success rate: {total_success}/{num_episodes} ({100 * total_success / num_episodes:.1f}%)")
    print(f"Completion rate: {total_complete}/{num_episodes} ({100 * total_complete / num_episodes:.1f}%)")
    print(f"Average steps: {avg_steps:.1f}")
    print(f"Total collisions: {total_collisions}")
    print("=" * 50)


if __name__ == "__main__":
    main()
