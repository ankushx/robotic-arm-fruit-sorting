#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveIt Motion Planning Controller Interface
Communicates with ROS MoveIt commander for collision-free trajectory generation and execution.
"""

import sys
import math

try:
    import rospy
    import moveit_commander
    from geometry_msgs.msg import Pose, PoseStamped
except ImportError:
    rospy = None
    moveit_commander = None


class MoveItController:
    """ROS MoveIt Commander wrapper for 6-DOF DOFBOT."""

    def __init__(self, group_name="dofbot_arm"):
        self.is_ros_available = rospy is not None and moveit_commander is not None
        self.group = None
        self.robot = None
        self.scene = None

        if self.is_ros_available:
            try:
                moveit_commander.roscpp_initialize(sys.argv)
                self.robot = moveit_commander.RobotCommander()
                self.scene = moveit_commander.PlanningSceneInterface()
                self.group = moveit_commander.MoveGroupCommander(group_name)

                self.group.set_planning_time(3.0)
                self.group.set_num_planning_attempts(5)
                self.group.set_goal_position_tolerance(0.01)
                self.group.set_goal_orientation_tolerance(0.05)
                self.group.set_max_velocity_scaling_factor(0.6)
                self.group.set_max_acceleration_scaling_factor(0.6)
                print(f"[MoveIt] Initialized planning group: {group_name}")
            except Exception as e:
                print(f"[MoveIt] Notice: Could not connect to MoveIt ROS core: {e}")
                self.group = None
        else:
            print("[MoveIt] Running without active ROS/MoveIt environment (Hardware direct / standalone mode).")

    def plan_and_execute_joint_target(self, joint_angles_deg):
        """
        Send joint target (in degrees) to MoveIt trajectory planner.
        """
        if self.group is None:
            return False

        joint_rad = [math.radians(a) for a in joint_angles_deg]
        self.group.set_joint_value_target(joint_rad)
        success = self.group.go(wait=True)
        self.group.stop()
        return success

    def plan_and_execute_cartesian_target(self, x, y, z):
        """
        Move end-effector to Cartesian pose (meters).
        """
        if self.group is None:
            return False

        pose_target = Pose()
        pose_target.position.x = x / 1000.0  # mm to meters
        pose_target.position.y = y / 1000.0
        pose_target.position.z = z / 1000.0
        pose_target.orientation.w = 1.0

        self.group.set_pose_target(pose_target)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return success
