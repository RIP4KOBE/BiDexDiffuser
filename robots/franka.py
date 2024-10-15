from typing import Dict
import numpy as np
from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from typing import Union
from robots.robot import Robot
from deoxys.utils.log_utils import get_deoxys_example_logger
from deoxys.utils.config_utils import (get_default_controller_config,verify_controller_config)
from examples.osc_control import (osc_move, move_to_target_pose)
logger = get_deoxys_example_logger()
import rospy
class FrankaRobot(Robot):
    """A class representing a Franka robot."""

    def __init__(
        self,
        which_arm: str = "",
        no_gripper: bool = False,
        gripper_type="",
        gripper_dim: int = 2
    ):
        """Initialize the Franka robot and its gripper."""
        [print("in franka robot:", which_arm) for _ in range(3)]
        if not no_gripper:
            assert gripper_type in ["soft_hand", "franka_gripper"], "Gripper type must be either 'soft_hand' or 'franka_gripper'"
            if gripper_type == "soft_hand":
                from robots.softhand_gripper import SoftHandGripper
                self.gripper_dim = 2
                if which_arm == "right":
                    soft_hand_device_id = 1
                    self.robot_interface = FrankaInterface(
                        config_root + "/charmander_right.yml", has_gripper=not no_gripper, use_visualizer=False,
                        gripper_type=gripper_type
                    )
                    logger.debug(["The ", which_arm, " robot arm interface has been initalized."])
                else:
                    soft_hand_device_id = 2
                    self.robot_interface = FrankaInterface(
                        config_root + "/charmander_left.yml", has_gripper=not no_gripper, use_visualizer=False,
                        gripper_type=gripper_type
                    )
                    logger.debug(["The ", which_arm, " robot arm interface has been initalized."])
                self.gripper = SoftHandGripper(soft_hand_device_id)
                self.gripper.start_subscriber()
            elif gripper_type == "franka_gripper":
                self.gripper_dim = 1
                self.robot_interface = FrankaInterface(
                    config_root + "/charmander_"+which_arm+".yml", has_gripper=not no_gripper, use_visualizer=False,
                    gripper_type=gripper_type
                )
        else:
            if which_arm == "left":
                self.robot_interface = FrankaInterface(
                    config_root + "/charmander_left.yml", has_gripper=not no_gripper, use_visualizer=False
                )
                logger.debug(["The ", which_arm, " robot arm interface has been initalized."])
            elif which_arm == "right":
                self.robot_interface = FrankaInterface(
                    config_root + "/charmander_right.yml", has_gripper=not no_gripper, use_visualizer=False
                )
                logger.debug(["The ", which_arm, "robot arm interface has been initalized."])
            else:
                raise ValueError("which_arm should be either 'left' or 'right'")

        [print("connect ", which_arm, "robot arm") for _ in range(3)]

        self._use_gripper = not no_gripper
        self.gripper_type = gripper_type

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            if self.gripper_type == "soft_hand":
                return 9
            else:
                return 8
        else:
            return 7

    def _get_gripper_pos(self) -> np.ndarray:
        """Get the current state of the Gripper.

        Returns:
            T: The current state of the Gripper.
        """
        if self.gripper_type == "softhand":
            gripper_pos = np.array(self.gripper.get_current_position())
            return gripper_pos
        else:
            gripper_pos = self.robot_interface.last_gripper_q
            return gripper_pos

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = np.array(self.robot_interface.last_q)
        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        return pos

    def get_joint_velocities(self) -> np.ndarray:
        return self.robot_interface.last_q_d

    # Deoxy下缺少TCPspeed获取接口，但是也许可以通过修改回调函数以100Hz频率速度粗略估算
    # def get_eef_speed(self) -> np.ndarray:
    #     return self.r_inter.getActualTCPSpeed()

    def get_eef_pose(self) -> np.ndarray:
        """Get the current pose of the leader robot's end effector.

        Returns:
            T: The current pose of the leader robot's end effector.
        """
        return self.robot_interface.last_eef_pose

    def command_joint_state(self, des_joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.
        Args:
            des_joint_state (np.ndarray): The state to command the leader robot to.
        """
        controller_type = "JOINT_IMPEDANCE"
        controller_cfg = get_default_controller_config(controller_type)
        action = des_joint_state
        pub_gripper_flag = True
        while True:
            if len(self.robot_interface._state_buffer) > 0:
                logger.info(f"Current Robot joint: {np.round(self.robot_interface.last_q, 3)}")
                logger.info(f"Desired Robot joint: {np.round(self.robot_interface.last_q_d, 3)}")
                if (
                    np.max(
                        np.abs(
                            np.array(self.robot_interface._state_buffer[-1].q)
                            - np.array(des_joint_state)
                        )
                    )
                    < 1e-3
                ):
                    break
            self.robot_interface.control(
                controller_type=controller_type,
                action=action,
                controller_cfg=controller_cfg,
            )
            if pub_gripper_flag and self.gripper_type == "softhand":
                self.gripper.move(des_joint_state[-2:])
                pub_gripper_flag = False

    def command_eef_pose(self, eef_pose: np.ndarray, num_step: int = 80) -> None:
        # TODO: The current control method is still incremental position control.
        controller_type = "OSC_POSE"
        controller_cfg = get_default_controller_config(controller_type)
        move_to_target_pose(
            self.robot_interface,
            controller_type,
            controller_cfg,
            target_delta_pose=eef_pose,
            num_steps=num_step,
            num_additional_steps=40,
            interpolation_method="linear",
        )
    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = self.get_eef_pose()
        gripper_pos = self._get_gripper_pos()
        return {
            "joint_positions": joints,
            "ee_pos_quat": pos_quat,  # TODO: this is pos_rot(4×4) actually
            "gripper_position": gripper_pos,
        }


