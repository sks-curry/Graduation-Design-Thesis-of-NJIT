import sys
import math
import copy
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from builtin_interfaces.msg import Time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SimpleRRTNode(Node):
    def __init__(self):
        super().__init__('simple_rrt_node')

        # 1. 基础参数配置
        self.group_name = "rm_group"
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.planner_id = "RRTConnectkConfigDefault"

        # 插值参数
        self.interpolation_joint_step = 0.05
        self.minimum_segment_duration = 0.15

        # 2. 初始化 Action 客户端
        self.get_logger().info("正在连接 MoveIt 和 Gazebo 控制器...")
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.exec_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/rm_group_controller/follow_joint_trajectory'
        )

        if not self.move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "连接 MoveIt 失败！未检测到 '/move_action' 服务。请检查 MoveIt 是否已完全启动。"
            )
            sys.exit(1)

        if not self.exec_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "连接底层控制器失败！未检测到 '/rm_group_controller/follow_joint_trajectory'。"
            )
            sys.exit(1)

        self.get_logger().info("连接成功！控制台已准备就绪。")

    def run_robot(self, target_angles_rad):
        """核心主流程：规划 -> 三次多项式插值 -> 绘制速度/加速度曲线 -> 执行"""
        print(
            f"[DEBUG] 目标关节角，单位 rad: {[round(a, 3) for a in target_angles_rad]}",
            flush=True
        )

        try:
            # --- 第一步：请求 MoveIt 进行 RRTConnect 规划 ---
            goal_msg = self.build_move_group_goal(target_angles_rad)
            self.get_logger().info(f"正在调用 {self.planner_id} 算法进行规划...")

            send_goal_future = self.move_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().error("规划请求被 MoveIt 拒绝！可能目标点超出关节限位。")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result

            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(
                    f"规划失败，错误码: {result.error_code.val}。请检查是否有碰撞或无解。"
                )
                return

            raw_trajectory = result.planned_trajectory.joint_trajectory
            self.get_logger().info(
                f"规划成功！原始稀疏轨迹包含 {len(raw_trajectory.points)} 个点。"
            )

            # --- 第二步：有约束三次多项式插值 ---
            dense_trajectory = self.interpolate_trajectory(raw_trajectory)
            self.get_logger().info(
                f"三次多项式插值完成！稠密轨迹包含 {len(dense_trajectory.points)} 个点。"
            )

            # --- 第三步：绘制速度和加速度变化图 ---
            self.save_velocity_acceleration_curves(dense_trajectory)

            # --- 第四步：下发给 Gazebo 执行 ---
            self.execute_in_gazebo(dense_trajectory)

        except Exception as e:
            self.get_logger().error(f"机械臂运行过程中发生未捕获的严重异常: {e}")

    def build_move_group_goal(self, target_angles_rad):
        """构造 MoveIt 规划请求"""
        constraints = Constraints()

        for name, angle in zip(self.joint_names, target_angles_rad):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(angle)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.planner_id = self.planner_id
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]

        # MoveIt 只负责规划，不直接执行
        goal.planning_options.plan_only = True

        return goal

    def interpolate_trajectory(self, raw_trajectory):
        """有速度约束的三次多项式轨迹插值"""
        interpolated = JointTrajectory()
        interpolated.header = raw_trajectory.header
        interpolated.joint_names = raw_trajectory.joint_names

        if not raw_trajectory.points:
            return interpolated

        raw_points = raw_trajectory.points
        joint_count = len(raw_points[0].positions)

        # 1. 提取原始轨迹点的位置
        raw_positions = [
            list(point.positions)
            for point in raw_points
        ]

        # 2. 构造单调递增的原始时间序列
        raw_times = [self.duration_to_sec(raw_points[0].time_from_start)]

        for i in range(1, len(raw_points)):
            current_time = self.duration_to_sec(raw_points[i].time_from_start)
            previous_time = raw_times[-1]

            if current_time - previous_time <= 1e-6:
                current_time = previous_time + self.minimum_segment_duration

            raw_times.append(current_time)

        # 3. 为每个原始轨迹点估计速度
        # 起点和终点速度设为 0，中间点速度用相邻点估计
        waypoint_velocities = []

        for i in range(len(raw_points)):
            if i == 0 or i == len(raw_points) - 1:
                waypoint_velocities.append([0.0] * joint_count)
            else:
                dt = raw_times[i + 1] - raw_times[i - 1]

                if dt <= 1e-6:
                    waypoint_velocities.append([0.0] * joint_count)
                else:
                    velocity = [
                        (raw_positions[i + 1][j] - raw_positions[i - 1][j]) / dt
                        for j in range(joint_count)
                    ]
                    waypoint_velocities.append(velocity)

        # 4. 添加起点
        first_point = JointTrajectoryPoint()
        first_point.positions = list(raw_positions[0])
        first_point.velocities = list(waypoint_velocities[0])
        first_point.accelerations = [0.0] * joint_count
        first_point.time_from_start = self.sec_to_duration(raw_times[0])
        interpolated.points.append(first_point)

        # 5. 逐段三次多项式插值
        for i in range(len(raw_points) - 1):
            q0 = raw_positions[i]
            q1 = raw_positions[i + 1]
            v0 = waypoint_velocities[i]
            v1 = waypoint_velocities[i + 1]

            t0 = raw_times[i]
            t1 = raw_times[i + 1]
            T = t1 - t0

            if T <= 1e-6:
                T = self.minimum_segment_duration

            max_delta = max(
                abs(q1[j] - q0[j])
                for j in range(joint_count)
            )

            steps = max(
                1,
                int(math.ceil(max_delta / self.interpolation_joint_step))
            )

            # 三次多项式系数
            # q(t)=a0+a1*t+a2*t^2+a3*t^3
            a0 = q0
            a1 = v0

            a2 = [
                3.0 * (q1[j] - q0[j]) / (T ** 2)
                - (2.0 * v0[j] + v1[j]) / T
                for j in range(joint_count)
            ]

            a3 = [
                2.0 * (q0[j] - q1[j]) / (T ** 3)
                + (v0[j] + v1[j]) / (T ** 2)
                for j in range(joint_count)
            ]

            for step in range(1, steps + 1):
                ratio = step / float(steps)
                local_t = ratio * T

                point = JointTrajectoryPoint()

                point.positions = [
                    a0[j]
                    + a1[j] * local_t
                    + a2[j] * (local_t ** 2)
                    + a3[j] * (local_t ** 3)
                    for j in range(joint_count)
                ]

                point.velocities = [
                    a1[j]
                    + 2.0 * a2[j] * local_t
                    + 3.0 * a3[j] * (local_t ** 2)
                    for j in range(joint_count)
                ]

                point.accelerations = [
                    2.0 * a2[j]
                    + 6.0 * a3[j] * local_t
                    for j in range(joint_count)
                ]

                point.time_from_start = self.sec_to_duration(
                    t0 + local_t
                )

                interpolated.points.append(point)

        return interpolated

    def save_velocity_acceleration_curves(self, trajectory):
        """绘制三次多项式插值轨迹的速度和加速度变化曲线"""
        points = trajectory.points

        if len(points) < 2:
            self.get_logger().warn("轨迹点过少，无法绘制速度和加速度曲线。")
            return

        times = [
            self.duration_to_sec(point.time_from_start)
            for point in points
        ]

        velocities = [
            list(point.velocities)
            for point in points
        ]

        accelerations = [
            list(point.accelerations)
            for point in points
        ]

        if not velocities or not accelerations:
            self.get_logger().warn("轨迹中缺少速度或加速度数据，无法绘图。")
            return

        joint_count = len(velocities[0])

        output_dir = os.path.join(os.getcwd(), "trajectory_plots")
        os.makedirs(output_dir, exist_ok=True)

        def draw_curve(value_data, ylabel, title, file_name):
            plt.figure(figsize=(10, 6))

            for j in range(joint_count):
                joint_data = [
                    value[j]
                    for value in value_data
                ]

                plt.plot(
                    times,
                    joint_data,
                    label=f"joint{j + 1}"
                )

            plt.xlabel("Time / s")
            plt.ylabel(ylabel)
            plt.title(title)
            plt.grid(True)
            plt.legend()
            plt.tight_layout()

            file_path = os.path.join(output_dir, file_name)
            plt.savefig(file_path, dpi=300)
            plt.close()

            self.get_logger().info(f"{title} 已保存：{file_path}")

        draw_curve(
            velocities,
            "Velocity / rad/s",
            "三次多项式插值轨迹速度曲线",
            "cubic_interpolation_velocity_curve.png"
        )

        draw_curve(
            accelerations,
            "Acceleration / rad/s^2",
            "三次多项式插值轨迹加速度曲线",
            "cubic_interpolation_acceleration_curve.png"
        )

    def execute_in_gazebo(self, trajectory):
        """发送轨迹到 Gazebo 控制器执行"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = copy.deepcopy(trajectory)

        # 置空时间戳，避免仿真时间不同步导致控制器拒绝执行
        goal.trajectory.header.stamp = Time()

        self.get_logger().info("正在将轨迹下发至 Gazebo 控制器...")

        send_goal_future = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("控制器拒绝执行轨迹！请检查控制器状态。")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("轨迹执行完毕！\n")

    @staticmethod
    def duration_to_sec(duration_msg):
        return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9

    @staticmethod
    def sec_to_duration(seconds):
        whole = int(seconds)
        nanos = int((seconds - whole) * 1e9)
        return Duration(seconds=whole, nanoseconds=nanos).to_msg()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SimpleRRTNode()
    except SystemExit:
        rclpy.shutdown()
        return

    print("\n===  RM65 RRT 轨迹规划控制台  ===")
    print("请输入 6 个关节目标【角度】，用空格分隔。")
    print("例如: 30 -45 90 0 60 0")
    print("输入 'q' 退出\n")

    while rclpy.ok():
        try:
            user_input = input("joint_target (deg)> ").strip()

            if user_input.lower() in ['q', 'quit', 'exit']:
                break

            if not user_input:
                continue

            tokens = user_input.split()

            if len(tokens) != 6:
                print("错误：请确切输入 6 个数字！")
                continue

            target_angles_rad = [
                math.radians(float(x))
                for x in tokens
            ]

            node.run_robot(target_angles_rad)

        except ValueError:
            print("错误：包含无效数字，请重新输入。")

        except KeyboardInterrupt:
            break

        except Exception as e:
            print(f"\n[ERROR] 主循环发生未预期异常: {e}")
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()