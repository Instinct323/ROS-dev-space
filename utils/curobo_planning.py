from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch
import trimesh
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo.util_file import join_path, load_yaml, get_world_configs_path, get_robot_configs_path
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig, IKResult
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

# cuda settings
torch.backends.cudnn.benchmark = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cuda.matmul.allow_tf32 = True


def clean_cached_properties(obj: object,
                            attributes: list[str]) -> object:
    """ Clear the cached properties of an object. """
    for attr in attributes:
        if attr in obj.__dict__: delattr(obj, attr)
    return obj


def load_cfg(cfg: str | Path | dict = None,
             cfg_path: str = None) -> dict:
    """ Load configuration from a file or dictionary. """
    if cfg is None: return {}

    if isinstance(cfg, str):
        if cfg_path and not Path(cfg).is_file():
            cfg = join_path(cfg_path, cfg)
        cfg = load_yaml(cfg)

    if isinstance(cfg, Path):
        cfg = load_yaml(str(cfg))
    return cfg


def to_ros_plan(plan: JointState,
                dt: float) -> "trajectory_msgs.msg.JointTrajectory":
    """ Convert a JointState plan to a ROS JointTrajectory message. """
    import rospy
    import trajectory_msgs.msg

    ros_msg = trajectory_msgs.msg.JointTrajectory()
    ros_msg.joint_names = plan.joint_names
    ros_msg.header.frame_id = ros_msg.joint_names[0].split("_")[0] + "_link0"

    ros_msg.points = [
        trajectory_msgs.msg.JointTrajectoryPoint(positions=p, velocities=v, accelerations=a, time_from_start=t)
        for p, v, a, t in
        zip(plan.position.tolist(), plan.velocity.tolist(), plan.acceleration.tolist(), map(rospy.Duration, np.arange(dt, len(plan), dt)))
    ]
    return ros_msg


@dataclass
class PlanVisConfig:
    n_vis: int = 10
    cmap: str = "rainbow"
    alpha_start: float = 0.3
    alpha_traj: float = 0.1
    alpha_end: float = 1.

    def __post_init__(self):
        assert self.n_vis >= 2

    def sample_indices(self, n: int) -> tuple[torch.Tensor, np.ndarray]:
        """ Sample indices for visualization. """
        i = torch.linspace(0, n - 1, self.n_vis).long()
        colors = plt.get_cmap(self.cmap)(i / i[-1])
        colors[0, -1], colors[1:-1, -1], colors[-1, -1] = self.alpha_start, self.alpha_traj, self.alpha_end
        return i, colors


@dataclass
class CuRoboPlanner:
    """ reference: https://curobo.org/ """
    robot_cfg: RobotConfig
    world_cfg: WorldConfig = None
    tensor_args: TensorDeviceType = field(default_factory=TensorDeviceType)

    # IK parameters
    rotation_threshold: float = 0.05
    position_threshold: float = 0.005

    # Motion generation parameters
    interpolation_dt: float = 0.1
    maximum_trajectory_time: float = None

    def __post_init__(self):
        robot_cfg = load_cfg(self.robot_cfg, cfg_path=get_robot_configs_path())
        self.robot_cfg = RobotConfig.from_dict(robot_cfg, tensor_args=self.tensor_args)

    def __setattr__(self, key, value):
        if key == "world_cfg":
            value = load_cfg(value, cfg_path=get_world_configs_path())
            value = WorldConfig.from_dict(value)

        super().__setattr__(key, value)
        if key in ("world_cfg",):
            clean_cached_properties(self, ("ik_solver", "motion_gen"))

    @cached_property
    def ik_solver(self) -> IKSolver:
        ik_cfg = IKSolverConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            tensor_args=self.tensor_args,
            rotation_threshold=self.rotation_threshold,
            position_threshold=self.position_threshold,
        )
        return IKSolver(ik_cfg)

    @property
    def kinematics(self) -> CudaRobotModel:
        return self.ik_solver.kinematics

    @cached_property
    def motion_gen(self) -> MotionGen:
        mg_cfg = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            tensor_args=self.tensor_args,
            rotation_threshold=self.rotation_threshold,
            position_threshold=self.position_threshold,
            interpolation_dt=self.interpolation_dt,
            maximum_trajectory_time=self.maximum_trajectory_time
        )
        mg = MotionGen(mg_cfg)
        # mg.warmup()
        return mg

    def plan(self,
             q_start: torch.Tensor,
             goal: Pose,
             vis_cfg: PlanVisConfig = None) -> JointState:
        """ Plan a trajectory from start to goal. """
        q_start = self.tensor_args.to_device(q_start)
        goal = goal.to(self.tensor_args)
        
        result = self.motion_gen.plan_single(JointState.from_position(q_start), goal)
        if result.success.item():
            plan = result.get_interpolated_plan()

            if vis_cfg:
                i, colors = vis_cfg.sample_indices(len(plan))

                # generate scene
                scene: list[WorldConfig] = [self.scene_robot(q[None], as_trimesh=False) for q in plan.position[i]]
                for s, c in zip(scene, colors): s.add_color(c)
                scene.insert(0, scene.pop())
                scene.append(self.scene_world(as_trimesh=False))

                # to trimesh
                mesh: trimesh.Scene = trimesh.util.concatenate([WorldConfig.get_scene_graph(s) for s in scene])
                mesh.show()

            return plan

    def refine_joint_state(self,
                           q: torch.Tensor) -> torch.Tensor:
        """ Refine a joint state by solving IK on its FK result. """
        assert q.shape == (1, self.kinematics.dof)
        q = self.tensor_args.to_device(q)
        goal = planner.solve_fk(q)
        result = self.solve_ik(goal)
        if result.success.item(): return result.solution[0]

    @property
    def retract_config(self) -> torch.Tensor:
        return self.motion_gen.get_retract_config()

    def sample_states(self,
                      n: int) -> torch.Tensor:
        return self.ik_solver.sample_configs(n)

    def scene_robot(self,
                    q: torch.Tensor,
                    as_trimesh: bool = True) -> WorldConfig | trimesh.Scene:
        q = self.tensor_args.to_device(q)
        try:
            geo = self.kinematics.get_robot_as_mesh(q)
        except ValueError:
            geo = self.kinematics.get_robot_as_spheres(q)[0]
        scene = WorldConfig(sphere=geo)
        return WorldConfig.get_scene_graph(scene) if as_trimesh else scene

    def scene_world(self,
                    as_trimesh: bool = True) -> WorldConfig | trimesh.Scene:
        scene = self.world_cfg
        return WorldConfig.get_scene_graph(scene) if as_trimesh else scene

    def solve_fk(self,
                 q: torch.Tensor) -> Pose:
        """ Forward kinematics """
        q = self.tensor_args.to_device(q)
        kin_state = self.ik_solver.fk(q)
        return Pose(position=kin_state.ee_position, quaternion=kin_state.ee_quaternion)

    def solve_ik(self,
                 goal: Pose) -> IKResult:
        """ Inverse kinematics """
        goal = goal.to(self.tensor_args)
        return self.ik_solver.solve_batch(goal)


if __name__ == '__main__':
    planner = CuRoboPlanner(robot_cfg="franka.yml")
    planner.world_cfg = "collision_cage.yml"

    for i in range(1):
        q = planner.sample_states(2)
        goal = planner.solve_fk(q[1:2])

        js = planner.plan(q[0:1], goal, vis_cfg=PlanVisConfig())
        ros_plan = to_ros_plan(js, dt=planner.interpolation_dt)
        print(ros_plan)
