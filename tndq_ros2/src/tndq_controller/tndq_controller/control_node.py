"""TNDQ ROS2 控制节点 —— 官方 rebotarm 透传通道上的力矩级闭环服务壳。

依赖链（导入顺序铁律见 paths.py）：main() 先 bootstrap() 完成 TNDQ_real
sys.path 引导与真机参数覆写，随后才导入 run_lib / ros_backend。

服务（std_srvs/Trigger，均幂等安全）：
    ~/bringup   POS_VEL 慢速就位 Q_INIT（闭环前置；官方默认 pos_vel 模式，
                本服务循环下发组级位置目标直至收敛，镜像 posvel_goto）
    ~/start     反馈健康闸门 -> 官方 set_mode("mit") -> 发送线程保持帧
                接力 -> 启动 run_lib.run_tndq_experiment 实验线程
                （task/duration/traj_backend 参数见 config yaml）
    ~/stop      请求优雅终止实验（step() 触发 KeyboardInterrupt，CSV
                保存到当前步，发送线程回保持期）
    ~/shutdown  stop -> 后端收尾（保持 0.3 s -> set_mode("pos_vel")，
                官方位置环托臂，全程不失能）

遥测：
    ~/state     JointState（q / qd / tau_sent，URDF 系，20 Hz）
    ~/status    String JSON（phase/task/running/aborted/fb_age，2 Hz）

安全语义与 TNDQ_real 直连版一致：闭环全程 MIT、就位/收尾走官方
pos_vel 位置环、永不自动失能；SIGINT（Ctrl+C）等价 ~/shutdown。
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from std_srvs.srv import Trigger

from .paths import bootstrap


class TNDQControllerNode(Node):
    def __init__(self) -> None:
        bootstrap()                              # TNDQ_real 引导（铁律顺序，幂等；
                                                 #   必须在 ros_backend 导入前）
        super().__init__("tndq_controller")
        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()

        # ---- ROS 参数（安全数值不在此重复定义：WATCHDOG/HOLD/斜坡等
        #      全部以 TNDQ_real/config/params_real.py 为单一真相源）----
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter(
            "joint_names",
            [f"joint{i}" for i in range(1, 7)])
        self.declare_parameter("send_rate_hz", 200.0)
        self.declare_parameter("fb_stale_timeout", 0.05)
        self.declare_parameter("task", "hold")          # hold|short|goto
        self.declare_parameter("duration", 30.0)
        self.declare_parameter("traj_backend", "pinocchio")  # goto 用
        self.declare_parameter("csv_dir", "~/tndq_ros2_results")
        self.declare_parameter("telemetry_rate", 20.0)

        ns = str(self.get_parameter("arm_namespace").value).strip("/")
        joint_names = [str(v) for v in
                       self.get_parameter("joint_names").value]
        send_rate = float(self.get_parameter("send_rate_hz").value)
        fb_stale = float(self.get_parameter("fb_stale_timeout").value)
        self.task = str(self.get_parameter("task").value)
        self.duration = float(self.get_parameter("duration").value)
        self.traj_backend = str(self.get_parameter("traj_backend").value)
        csv_dir = Path(self.get_parameter("csv_dir").value).expanduser()
        telemetry_rate = float(self.get_parameter("telemetry_rate").value)

        # ---- 后端（bootstrap 已完成，config.* 可导入）----
        from .ros_backend import RosChannelBackend
        self.backend = RosChannelBackend(
            self, arm_namespace=ns, joint_names=joint_names,
            send_rate_hz=send_rate, fb_stale_timeout=fb_stale)

        # ---- 实验线程状态 ----
        self._exp_thread: threading.Thread | None = None
        self._exp_summary: dict | None = None
        self._exp_error: str | None = None
        self._exp_task = ""

        # ---- 服务 ----
        for name, cb in (("bringup", self._on_bringup),
                         ("start", self._on_start),
                         ("stop", self._on_stop),
                         ("shutdown", self._on_shutdown)):
            self.create_service(Trigger, f"~/{name}", cb,
                                callback_group=self.reentrant_group)

        # ---- 遥测 ----
        from sensor_msgs.msg import JointState
        self._state_pub = self.create_publisher(
            JointState, "~/state", qos_profile_sensor_data)
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self._joint_names = joint_names
        period = 1.0 / max(telemetry_rate, 1.0)
        self._telemetry = self.create_timer(
            period, self._publish_telemetry,
            callback_group=self.reentrant_group)
        self._status_timer = self.create_timer(
            0.5, self._publish_status,
            callback_group=self.reentrant_group)

        self.get_logger().info(
            f"tndq_controller 就绪：官方通道 /{ns}/*，任务={self.task} "
            f"时长={self.duration}s；流程：bringup -> start -> (stop) -> "
            "shutdown")

    # ------------------------------------------------------------------
    # 任务构建（复用 run_lib 轨迹构建器；hold 用常值笛卡尔腿）
    # ------------------------------------------------------------------

    def _build_trajectory(self, task: str, duration: float):
        from config.params import Q_INIT
        from experiments import run_lib

        if task == "hold":
            chain = run_lib.B601TCPChain(run_lib.B601_DH_TABLE)
            x0 = chain.fkm(Q_INIT)
            p0 = run_lib.dq_translation(x0)
            r0 = run_lib.dq_rotation(x0)
            from simdata.traj_kinematic import (CartesianWaypointTrajectory,
                                                KinematicTrajectoryAdapter)
            wp = CartesianWaypointTrajectory(
                (p0, r0), [(p0, r0, float(duration), 0.0)])
            return KinematicTrajectoryAdapter(wp)
        if task == "short":
            traj, t_move, _ = run_lib.build_short_trajectory()
            if duration < t_move:
                self.get_logger().warn(
                    f"duration={duration}s < short 轨迹 {t_move:.1f}s，"
                    "将在轨迹中途中止")
            return traj
        if task == "goto":
            if self.traj_backend == "pinocchio":
                # pinocchio 缺失时函数内部自动降级 kinematic
                traj, t_move, _ = \
                    run_lib.build_setpoint_goto_trajectory_pinocchio()
            else:
                traj, t_move, _ = \
                    run_lib.build_setpoint_goto_trajectory_kinematic()
            if duration < t_move:
                self.get_logger().warn(
                    f"duration={duration}s < goto 轨迹 {t_move:.1f}s，"
                    "将在轨迹中途中止")
            return traj
        raise ValueError(f"未知 task: {task!r}（可选 hold|short|goto）")

    # ------------------------------------------------------------------
    # 服务回调
    # ------------------------------------------------------------------

    def _experiment_alive(self) -> bool:
        return (self._exp_thread is not None
                and self._exp_thread.is_alive())

    def _on_bringup(self, _req, resp):
        """POS_VEL 慢速就位 Q_INIT（镜像 posvel_goto；须在 MIT 之前）。"""
        from config.params import Q_INIT
        try:
            if self._experiment_alive():
                raise RuntimeError("实验运行中，先 stop")
            if self.backend._mit_active:
                raise RuntimeError(
                    "已处于 MIT 闭环（先 shutdown 收尾再 bringup）")
            vlim, tol, timeout = 0.3, 0.01, 60.0
            t0 = time.monotonic()
            converged = False
            while time.monotonic() - t0 < timeout:
                self.backend.publish_posvel(Q_INIT, vlim=vlim)
                time.sleep(0.1)
                q, _ = self.backend.get_joint_state()
                if float(np.max(np.abs(q - Q_INIT))) < tol:
                    time.sleep(1.5)              # 静置稳定
                    q, _ = self.backend.get_joint_state()
                    err = float(np.max(np.abs(q - Q_INIT)))
                    if err < tol:
                        converged = True
                        break
            if not converged:
                raise RuntimeError(
                    f"bringup 超时（{timeout}s 未收敛到 Q_INIT "
                    f"±{tol} rad）——检查官方包是否 pos_vel 模式且已使能")
            resp.success = True
            resp.message = f"就位完成 max|dq|={err:.4f} rad"
        except Exception as exc:                 # noqa: BLE001
            resp.success = False
            resp.message = str(exc)
        self.get_logger().info(f"bringup: {resp.message}")
        return resp

    def _on_start(self, _req, resp):
        try:
            if self._experiment_alive():
                raise RuntimeError("实验已在运行")
            self._exp_error = None
            self._exp_summary = None
            self.backend.setup()
            traj = self._build_trajectory(self.task, self.duration)
            csv_dir = Path(
                self.get_parameter("csv_dir").value).expanduser()
            csv_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            csv_path = str(csv_dir / f"exp_ros_{self.task}_{stamp}.csv")
            from experiments import run_lib
            self._exp_task = self.task
            self._exp_thread = threading.Thread(
                target=self._experiment_main,
                args=(run_lib, self.backend, traj, self.duration,
                      csv_path),
                name="tndq-experiment", daemon=True)
            self._exp_thread.start()
            resp.success = True
            resp.message = (f"实验启动 task={self.task} "
                            f"duration={self.duration}s csv={csv_path}")
        except Exception as exc:                 # noqa: BLE001
            resp.success = False
            resp.message = f"start 失败: {exc}"
        self.get_logger().info(resp.message)
        return resp

    def _experiment_main(self, run_lib, backend, traj, duration,
                         csv_path):
        try:
            self._exp_summary = run_lib.run_tndq_experiment(
                backend, traj, duration, csv_path, label="tndq_ros2")
        except Exception as exc:                 # noqa: BLE001
            self._exp_error = str(exc)
            self.get_logger().error(f"实验线程异常: {exc}")
        finally:
            self.get_logger().info("实验线程退出（发送线程保持期托臂）")

    def _on_stop(self, _req, resp):
        if not self._experiment_alive():
            resp.success, resp.message = False, "无运行中的实验"
        else:
            self.backend.request_stop()
            resp.success = True
            resp.message = "已请求优雅终止（CSV 保存到当前步，回保持期）"
        self.get_logger().info(resp.message)
        return resp

    def _on_shutdown(self, _req, resp):
        try:
            if self._experiment_alive():
                self.backend.request_stop()
                self._exp_thread.join(timeout=5.0)
            self.backend.close()
            resp.success = True
            resp.message = "已收尾：官方 pos_vel 位置保持接管（未失能）"
        except Exception as exc:                 # noqa: BLE001
            resp.success = False
            resp.message = f"shutdown 异常: {exc}"
        self.get_logger().info(resp.message)
        return resp

    # ------------------------------------------------------------------
    # 遥测
    # ------------------------------------------------------------------

    def _publish_telemetry(self) -> None:
        from sensor_msgs.msg import JointState
        snap = self.backend.snapshot()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [float(v) for v in snap["q"]]
        msg.velocity = [float(v) for v in snap["qd"]]
        msg.effort = [float(v) for v in snap["tau_sent"]]
        self._state_pub.publish(msg)

    def _publish_status(self) -> None:
        snap = self.backend.snapshot()
        status = {
            "phase": snap["phase"],
            "ramp_alpha": round(snap["ramp_alpha"], 3),
            "fb_age_ms": (round(snap["fb_age_ms"], 1)
                          if snap["fb_age_ms"] == snap["fb_age_ms"] else -1),
            "task": self._exp_task,
            "running": self._experiment_alive(),
            "error": self._exp_error,
            "aborted": (self._exp_summary or {}).get("aborted"),
            "pos_err_final": (self._exp_summary or {}).get("pos_err_final"),
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """SIGINT 等价 ~/shutdown：优雅终止实验 -> 后端收尾（不失能）。"""
        try:
            if self._experiment_alive():
                self.backend.request_stop()
                self._exp_thread.join(timeout=5.0)
            if self.backend._mit_active:
                self.backend.close()
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().error(f"shutdown 异常: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TNDQControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
