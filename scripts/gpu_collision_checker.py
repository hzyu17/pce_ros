#!/usr/bin/env python3
"""
GPU Collision Checker using CuRobo
Minimal wrapper for batch collision checking
"""

import torch
import numpy as np
import rospy
from typing import Dict, List, Tuple

try:
    from curobo.geom.types import WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import RobotConfig
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
    CUROBO_AVAILABLE = True
except ImportError:
    CUROBO_AVAILABLE = False
    rospy.logwarn("CuRobo not available. GPU acceleration disabled.")


class GPUCollisionChecker:
    """GPU-accelerated collision checker using CuRobo"""
    
    def __init__(self, urdf_path: str, base_link: str, ee_link: str, 
                 collision_clearance: float = 0.05):
        """
        Initialize GPU collision checker.
        
        Args:
            urdf_path: Path to robot URDF file
            base_link: Robot base link name
            ee_link: End effector link name
            collision_clearance: Collision safety margin (epsilon)
        """
        if not CUROBO_AVAILABLE:
            raise RuntimeError("CuRobo not installed. Run install_gpu_deps.sh")
        
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA not available. GPU required for acceleration.")
        
        rospy.loginfo("Initializing GPU collision checker...")
        rospy.loginfo(f"  URDF: {urdf_path}")
        rospy.loginfo(f"  Device: {torch.cuda.get_device_name(0)}")
        
        # Setup CUDA device
        self.tensor_args = TensorDeviceType(
            device=torch.device("cuda:0"),
            dtype=torch.float32
        )
        
        # Load robot configuration
        robot_cfg = RobotConfig.from_basic(
            urdf_path,
            base_link=base_link,
            ee_link=ee_link,
            tensor_args=self.tensor_args
        )
        
        # Empty world config (will be updated later)
        self.world_cfg = WorldConfig.from_dict({"cuboid": {}})
        
        # Create motion generator (handles collision checking internally)
        config = MotionGenConfig.load_from_robot_config(
            robot_cfg,
            self.world_cfg,
            self.tensor_args,
        )
        self.motion_gen = MotionGen(config)
        
        self.collision_clearance = collision_clearance
        self.device = self.tensor_args.device
        
        rospy.loginfo("✓ GPU collision checker ready")
    
    def update_world(self, obstacles: Dict[str, Dict]) -> None:
        """
        Update obstacle world.
        
        Args:
            obstacles: Dictionary of obstacles in format:
                {
                    "table": {
                        "type": "cuboid",
                        "dims": [0.6, 1.0, 0.1],
                        "pose": [x, y, z, qw, qx, qy, qz]
                    },
                    ...
                }
        """
        cuboids = {}
        
        for name, obj in obstacles.items():
            if obj.get("type") == "cuboid":
                cuboids[name] = {
                    "dims": obj["dims"],
                    "pose": obj["pose"]
                }
        
        self.world_cfg = WorldConfig.from_dict({"cuboid": cuboids})
        self.motion_gen.update_world(self.world_cfg)
        
        rospy.loginfo(f"Updated GPU world with {len(cuboids)} obstacles")
    
    def check_trajectory_batch(self, trajectories: np.ndarray) -> np.ndarray:
        """
        Batch collision check for multiple trajectories.
        
        Args:
            trajectories: numpy array [num_samples, num_waypoints, num_joints]
        
        Returns:
            costs: numpy array [num_samples] with collision costs
        """
        # Convert to torch tensor
        traj_tensor = torch.from_numpy(trajectories).to(
            device=self.device,
            dtype=torch.float32
        )
        
        num_samples, num_waypoints, num_joints = traj_tensor.shape
        
        # Reshape to [num_samples * num_waypoints, num_joints]
        configs = traj_tensor.reshape(-1, num_joints)
        
        # Batch collision check on GPU
        with torch.no_grad():
            # Get collision distances for all configurations
            collision_query = self.motion_gen.collision_checker.check_robot_sphere_collisions(
                configs
            )
            distances = collision_query.distance  # [num_samples * num_waypoints]
        
        # Reshape to [num_samples, num_waypoints]
        distances = distances.reshape(num_samples, num_waypoints)
        
        # Compute costs using CHOMP-style cost function (vectorized on GPU)
        costs = self._compute_costs_gpu(distances)
        
        # Return as numpy array
        return costs.cpu().numpy()
    
    def _compute_costs_gpu(self, distances: torch.Tensor) -> torch.Tensor:
        """
        Compute collision costs from distances (GPU-accelerated).
        
        Implements CHOMP-style cost function:
        - distance < 0: inside obstacle, cost = -distance + 0.5 * epsilon
        - 0 <= distance < epsilon: near obstacle, cost = 0.5 * (1/epsilon) * (distance - epsilon)^2
        - distance >= epsilon: safe, cost = 0
        
        Args:
            distances: [num_samples, num_waypoints] tensor of distances
        
        Returns:
            costs: [num_samples] tensor of total costs
        """
        epsilon = self.collision_clearance
        
        # Initialize costs
        costs = torch.zeros_like(distances)
        
        # Inside obstacle (penetration)
        mask_inside = distances < 0
        costs[mask_inside] = -distances[mask_inside] + 0.5 * epsilon
        
        # Near obstacle (clearance violation)
        mask_clearance = (distances >= 0) & (distances < epsilon)
        diff = distances[mask_clearance] - epsilon
        costs[mask_clearance] = 0.5 * (1.0 / epsilon) * diff * diff
        
        # Sum costs across all waypoints for each trajectory
        total_costs = costs.sum(dim=1)  # [num_samples]
        
        return total_costs


# Simple test/demo
if __name__ == "__main__":
    rospy.init_node("gpu_collision_checker_test")
    
    # Example usage
    checker = GPUCollisionChecker(
        urdf_path="/path/to/robot.urdf",
        base_link="panda_link0",
        ee_link="panda_hand",
        collision_clearance=0.05
    )
    
    # Add obstacles
    checker.update_world({
        "table": {
            "type": "cuboid",
            "dims": [0.6, 1.0, 0.1],
            "pose": [0.5, 0.0, -0.05, 1, 0, 0, 0]
        }
    })
    
    # Test batch collision checking
    test_trajectories = np.random.randn(1000, 20, 7)  # 1000 samples, 20 waypoints, 7 joints
    costs = checker.check_trajectory_batch(test_trajectories)
    
    rospy.loginfo(f"Computed {len(costs)} collision costs")
    rospy.loginfo(f"Mean cost: {np.mean(costs):.4f}")
    rospy.loginfo(f"Max cost: {np.max(costs):.4f}")
