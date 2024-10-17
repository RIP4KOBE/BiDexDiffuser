import platform
import struct
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Pose, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
## Send Miscellanous Command to Ability Hand


def call_gripper(gripper, cmd, end_event, debug):
    i = 0
    while not end_event.is_set():
        gripper._serial_comm(cmd, debug=debug)
        time.sleep(0.01)
        # print("call ", i, end_event.is_set())
        i += 1


class SoftHandGripper:
    # default reply mode 0x11 stands for position mode variant 2
    def __init__(
        self,
        soft_hand_id: int = 1,
    ):
        self.reset_count = 0
        self.total_count = 0

        ## Data arrays for position
        ## Safe "last position" for hand to start at
        self.pos = [0] * 2
        self.prev_pos = self.pos.copy()
        self.last_pos_cmd = [0.0] * 2

        self.ros_node = rospy.init_node('Sub_softhand_gripper'+str(soft_hand_id), anonymous=True)
        self.rate = rospy.rate(125)

        ## Set device ID for soft hand
        assert soft_hand_id in [1, 2], "Soft Hand topic must be 1 or 2"
        self.softhand_grasp_pub = rospy.Publisher('/qbhand2m_chain/control/qbhand2m'+str(soft_hand_id) +
                                                  '_synergies_trajectory_controller/command',
                                                  JointTrajectory, queue_size=10)
        self.softhand_grasp_sub = rospy.Subscriber('/qbhand2m_chain/control/joint_states', JointState, self.update_soft_hand_state)
        self.softhand_id = soft_hand_id

    def update_soft_hand_state(self, msg):
        self.prev_pos = self.pos.copy()
        self.pos = [msg.synergies, msg.synergies2]

    def start_subscriber(self):
        rospy.spin()

    def create_ros_msg(self, synergy_control_parameters):
        traj_l = JointTrajectory()
        traj_l.header.stamp = rospy.Time.now()
        traj_l.header.frame_id = " "
        traj_l.joint_names = ["qbhand2m" + str(self.softhand_id) + "_manipulation_joint", "qbhand2m2_synergy_joint"]
        point = JointTrajectoryPoint()
        point.positions = [synergy_control_parameters[1], synergy_control_parameters[0]]
        point.time_from_start = rospy.Duration(1.5)
        traj_l.points.append(point)
        return traj_l

    def _serial_comm(self, pos_cmd, debug=False):
        """
        Send control command and read latest data.
        pos_cmd: an array of length 2 in range [0, 1]
        """
        ## Get message with new positions
        if pos_cmd is None:
            print("pos_cmd is None, it will be set randomly between [0,1].")
            pos_cmd = np.random.uniform(0, 1, size=2)
        self.last_pos_cmd = pos_cmd

        if debug:
            print("Last pos:", self.pos)
            print("Pos cmd:", pos_cmd)
        ros_traj_command_msg = self.create_ros_msg(pos_cmd)
        ## Send Message
        try:
            self.softhand_grasp_pub.publish(ros_traj_command_msg)
            rospy.loginfo("softhand "+str(self.softhand_id)+" motion execution...")
            # rospy.sleep(1.5)
            need_reset = False
        except:
            rospy.loginfo("softhand "+str(self.softhand_id)+" motion execution failed")
            need_reset = True
        if need_reset:
            self.reset_count += 1
            self.pos = self.prev_pos.copy()
        self.total_count += 1

    def close(self):
        self.softhand_grasp_pub.unregister()
        self.softhand_grasp_sub.unregister()

    def move(self, position, debug=False):
        position = np.ndarray(position)
        self._serial_comm(position, debug=debug)

    def get_current_state(self):
        # pos (2,)s
        return self.pos

    def get_current_position(self):
        # pos readings
        return self.pos
