from dataclasses import dataclass
from multiprocessing import Process
from typing import List, Optional, Tuple

import tyro

from camera_node import ZMQServerCamera, ZMQServerCameraFaster
from robot_node import ZMQServerRobot
from robots.robot import BimanualRobot

import rospy
from geometry_msgs.msg import Pose, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

@dataclass
class Args:
    robot: str = "curi"
    no_gripper = False
    hand_type: str = "soft_hand"
    hostname: str = "192.168.2.2"
    faster: bool = True
    cam_names: Tuple[str, ...] = "435"
    ability_gripper_grip_range: int = 110
    img_size: Optional[Tuple[int, int]] = None  # (320, 240)

# TODO: 相机模组仍待开发
def launch_server_cameras(port: int, camera_id: List[str], args: Args):
    from cameras.realsense_camera import RealSenseCamera

    camera = RealSenseCamera(camera_id, img_size=args.img_size)

    if args.faster:
        server = ZMQServerCameraFaster(camera, port=port, host=args.hostname)
    else:
        server = ZMQServerCamera(camera, port=port, host=args.hostname)
    print(f"Starting camera server on port {port}")
    server.serve()


def launch_robot_server(port: int, args: Args):
    if args.robot == "curi_single_arm":
        from robots.franka import FrankaRobot
        robot = FrankaRobot(which_arm="left",
                            no_gripper=args.no_gripper)
    elif args.robot == "bimanual_curi":
        from robots.franka import FrankaRobot
        if args.no_gripper == False:
            _robot_l = FrankaRobot(
                which_arm="left",
                no_gripper=args.no_gripper,
                gripper_type=args.hand_type,
                gripper_dim=2,
            )
            _robot_r = FrankaRobot(
                which_arm="right",
                no_gripper=args.no_gripper,
                gripper_type=args.hand_type,
                gripper_dim=2
            )
        else:
            # Franka_gripper
            _robot_l = FrankaRobot(which_arm="left", no_gripper=True)
            _robot_r = FrankaRobot(which_arm="right", no_gripper=True)
        robot = BimanualRobot(_robot_l, _robot_r)
    else:
        raise NotImplementedError(f"Robot {args.robot} not implemented")
    server = ZMQServerRobot(robot, port=port, host=args.hostname)
    print(f"Starting robot node")
    server.serve()


CAM_IDS = {
    "435": "000000000000",
}


def create_camera_server(args: Args) -> List[Process]:
    ids = [CAM_IDS[name] for name in args.cam_names]
    camera_port = 5000
    # start a single python process for all cameras
    print(f"Launching cameras {ids} on port {camera_port}")
    server = Process(target=launch_server_cameras, args=(camera_port, ids, args))
    return server


def main(args):
    # camera_server = create_camera_server(args)
    # print("Starting camera server process")
    # camera_server.start()

    launch_robot_server(6000, args)


if __name__ == "__main__":
    main(tyro.cli(Args))
