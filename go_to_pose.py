#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用代码控制 EDULITE A3 机械臂运动到指定位姿 / 指定关节角（MoveIt2 MoveGroup Action）

修复记录（2026-09-28）：
- 增加 req.start_state.is_diff = True：告诉 move_group 从机器人当前状态开始规划，
  避免空 start_state 被当成全零位姿，导致轨迹起点与实际不符、控制器拒绝执行(-4)。
- 显式设置 planning_options.plan_only = False：确保规划后自动执行。
- 显式设置 req.planner_id = 'RRTConnect'。
- main() 默认先测关节空间运动 go_to_joints（不依赖逆解），跑通后再开笛卡尔。
"""
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                             OrientationConstraint, JointConstraint,
                             PlanningOptions, BoundingVolume)
from shape_msgs.msg import SolidPrimitive


class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose')
        self._client = ActionClient(self, MoveGroup, '/move_action')

    def _send_request(self, request, plan_only=False):
        self.get_logger().info('等待 move_group 服务器...')
        self._client.wait_for_server()

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = plan_only   # False=规划并执行, True=只规划

        self.get_logger().info(
            f'发送请求: group={request.group_name}, '
            f'plan_only={plan_only}, planner={request.planner_id}'
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('目标被服务器拒绝')
            return False

        self.get_logger().info('目标已接受，等待规划+执行结果...')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        ec = result.error_code.val
        if ec == 1:   # SUCCESS
            self.get_logger().info('运动规划并执行成功！')
            return True
        elif ec == -4:
            self.get_logger().error(
                'CONTROL_FAILED(-4): 规划成功但控制器执行失败。\n'
                '  排查: 1) 确认 ros2 control list_controllers 中 arm_controller 为 active\n'
                '        2) 确认本脚本 start_state.is_diff=True 已生效\n'
                '        3) 先跑关节运动 go_to_joints 排除逆解问题'
            )
        else:
            self.get_logger().error(f'失败，错误码: {ec}')
        return False

    def go_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """末端 end_effector 运动到指定位置(x,y,z)和姿态(四元数)"""
        # 位置约束：用一个小球框住目标点，球半径=位置容差
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]                 # 位置容差 5mm（放宽，原来2mm太严）
        sphere_pose = Pose()
        sphere_pose.position.x = x
        sphere_pose.position.y = y
        sphere_pose.position.z = z
        sphere_pose.orientation.w = 1.0
        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(sphere_pose)

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'end_effector'
        pc.constraint_region = region
        pc.weight = 1.0

        # 姿态约束
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'end_effector'
        oc.orientation.x = qx
        oc.orientation.y = qy
        oc.orientation.z = qz
        oc.orientation.w = qw
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        constraint = Constraints()
        constraint.position_constraints.append(pc)
        constraint.orientation_constraints.append(oc)

        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.start_state.is_diff = True              # ★关键：从当前状态开始
        req.planner_id = 'RRTConnect'
        req.goal_constraints.append(constraint)
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        return self._send_request(req)

    def go_to_joints(self, values):
        """关节空间运动：values 为 L1~L6 六个关节的目标角度(弧度)"""
        names = ['L1_joint', 'L2_joint', 'L3_joint',
                 'L4_joint', 'L5_joint', 'L6_joint']
        constraint = Constraints()
        for name, v in zip(names, values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = v
            jc.tolerance_above = 0.05               # 放宽到 5 度容差
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraint.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.start_state.is_diff = True              # ★关键：从当前状态开始
        req.planner_id = 'RRTConnect'
        req.goal_constraints.append(constraint)
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        return self._send_request(req)


def main():
    rclpy.init()
    node = GoToPose()

    # ===== 第一步：先测关节空间运动（不依赖逆解，最容易跑通）=====
    # 从当前位置小幅移动：L2 从当前值动一点。
    # 如果当前在全零位，目标设为 [0, 0.5, -0.5, 0, 0, 0]（L2=约28度, L3=-28度）
    node.go_to_joints([0.0, 0.5, -0.5, 0.0, 0.0, 0.0])

    # ===== 第二步：关节跑通后，再开笛卡尔位姿 =====
    # node.go_to_pose(0.25, 0.0, 0.30)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
