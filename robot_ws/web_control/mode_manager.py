#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import signal
import time
import threading
from typing import List


class ModeManager(Node):
    def __init__(self):
        super().__init__("mode_manager")

        # ===== MODE SELECT =====
        self.mode_sub = self.create_subscription(String, "/robot_mode", self.mode_cb, 10)

        # ===== COMMAND CHANNEL =====
        self.stop_pub = self.create_publisher(String, "/manual_cmd", 10)
        self.explore_pub = self.create_publisher(String, "/explore_cmd", 10)
        self.ui_pub = self.create_publisher(String, "/ui_status", 10)

        # explore / goto command
        self.explore_cmd_sub = self.create_subscription(String, "/explore_cmd", self.on_explore_cmd, 10)
        self.goto_cmd_sub = self.create_subscription(String, "/goto_goal", self.on_goto_cmd, 10)

        self.procs: List[subprocess.Popen] = []
        self.current_mode = None  # "MANUAL" | "FOLLOW" | "AUTO_EXPLORE" | "SAVING_MAP" | "NAV_READY"

        self.get_logger().info("ModeManager READY")
        self.ui("Hệ thống sẵn sàng.")

    # =====================================================
    # UI helper
    # =====================================================
    def ui(self, message: str):
        msg = String()
        msg.data = message
        self.ui_pub.publish(msg)
        self.get_logger().info(f"[UI] {message}")

    # =====================================================
    # UTILITIES
    # =====================================================
    def start_proc(self, name: str, cmd: list):
        self.get_logger().info(f"[START] {name}: {' '.join(cmd)}")
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            p._name = name
            self.procs.append(p)
        except Exception as e:
            self.get_logger().error(f"Failed to start {name}: {e}")
            self.ui(f"❌ Không khởi động được {name}.")

    def kill_all_child(self):
        if not self.procs:
            return

        self.get_logger().warn("Killing all mode processes...")
        for p in self.procs:
            if p.poll() is None:
                try:
                    p.send_signal(signal.SIGINT)
                except Exception:
                    pass

        time.sleep(0.5)

        for p in self.procs:
            if p.poll() is None:
                try:
                    p.kill()
                except Exception:
                    pass

        self.procs.clear()
        self.get_logger().info("All processes terminated.")

    def stop_process_by_name(self, name: str):
        for p in self.procs:
            if getattr(p, "_name", "") == name and p.poll() is None:
                try:
                    self.get_logger().info(f"[STOP] {name}")
                    p.send_signal(signal.SIGINT)
                except Exception:
                    pass

    def kill_cameras(self):
        self.get_logger().warn("Killing ALL camera-related processes")
        subprocess.call(["pkill", "-f", "mjpg_streamer"], stderr=subprocess.DEVNULL)
        subprocess.call(["pkill", "-f", "web_video_server"], stderr=subprocess.DEVNULL)
        subprocess.call(["pkill", "-f", "v4l2_camera_node"], stderr=subprocess.DEVNULL)
        subprocess.call(["pkill", "-f", "follow_camera"], stderr=subprocess.DEVNULL)
        time.sleep(0.5)

    def send_stop_signal(self):
        msg = String()
        msg.data = "STOP"
        self.stop_pub.publish(msg)
        # IMPORTANT: do NOT publish STOP to /explore_cmd here (STOP loop risk)
        self.get_logger().info("STOP sent → robot stopped")

    # =====================================================
    # MODE HANDLER
    # =====================================================
    def mode_cb(self, msg: String):
        mode = msg.data.strip().upper()
        self.get_logger().info(f"[MODE] {mode}")
        self.ui(f"Nhận lệnh mode: {mode}")

        # STOP ALL
        if mode == "STOP_ALL":
            self.send_stop_signal()
            self.kill_all_child()
            self.kill_cameras()
            self.current_mode = None
            self.ui("Đã dừng toàn bộ.")
            return

        # MANUAL
        if mode == "MANUAL":
            self.kill_all_child()
            self.kill_cameras()

            self.start_proc("mjpg_streamer", [
                "mjpg_streamer",
                "-i", "input_uvc.so -d /dev/camera_main -r 640x480 -f 30",
                "-o", "output_http.so -p 8080"
            ])

            self.current_mode = "MANUAL"
            self.ui("MANUAL READY.")
            return

        # FOLLOW START
        if mode == "FOLLOW_START":
            self.kill_all_child()
            self.kill_cameras()

            self.start_proc("follow_camera", ["ros2", "launch", "follow_camera", "follow_camera.launch.py"])
            self.start_proc("ydlidar", ["ros2", "launch", "ydlidar_ros2_driver", "ydlidar_launch.py"])
            self.start_proc("target_nav", ["ros2", "launch", "project_target_nav", "project_target_nav.launch.py"])

            self.current_mode = "FOLLOW"
            self.ui("FOLLOW MODE READY.")
            return

        # FOLLOW STOP
        if mode == "FOLLOW_STOP":
            self.send_stop_signal()
            self.kill_all_child()
            self.kill_cameras()
            self.current_mode = None
            self.ui("Đã dừng FOLLOW.")
            return

        # AUTO EXPLORE
        if mode == "AUTO_EXPLORE":
            self.kill_all_child()
            self.kill_cameras()

            self.start_proc("ydlidar", ["ros2", "launch", "ydlidar_ros2_driver", "ydlidar_launch.py"])
            self.start_proc("laser_tf", ["ros2", "launch", "auto_explore", "laser_tf.launch.py"])
            self.start_proc("rf2o", ["ros2", "launch", "rf2o_laser_odometry", "rf2o_laser_odometry.launch.py"])

            self.current_mode = "AUTO_EXPLORE"
            self.ui("AUTO_EXPLORE READY. Bấm START để bắt đầu SLAM + explore.")
            return

        self.get_logger().warn(f"Unknown mode: {mode}")
        self.ui(f"⚠️ Mode không hợp lệ: {mode}")

    # =====================================================
    # EXPLORE COMMAND HANDLER
    # =====================================================
    def on_explore_cmd(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"[EXPLORE_CMD] {cmd}")

        if self.current_mode not in ("AUTO_EXPLORE", "SAVING_MAP", "NAV_READY"):
            self.get_logger().warn("Ignore explore_cmd (not in explore-related modes)")
            return

        if cmd == "START":
            if self.current_mode == "SAVING_MAP":
                self.ui("⏳ Đang lưu map, vui lòng đợi lưu xong.")
                return

            self.ui("Bắt đầu SLAM + Auto Explore...")
            # start slam (if not already)
            self.start_proc("slam", ["ros2", "launch", "auto_explore", "slam_online.launch.py"])
            self.start_proc("explore", ["ros2", "launch", "auto_explore", "auto_explore.launch.py"])
            self.current_mode = "AUTO_EXPLORE"
            return

        if cmd == "STOP":
            self.stop_process_by_name("explore")
            self.send_stop_signal()
            self.ui("Đã dừng Explore.")
            return

        if cmd == "SAVE":
            # Stop explore but keep SLAM running so TF map->odom still exists for navigation
            self.stop_process_by_name("explore")
            self.send_stop_signal()

            self.current_mode = "SAVING_MAP"
            self.ui("Đang lưu map...")

            def _save_job():
                out_prefix = "/home/cong19102003/maps/my_map"
                try:
                    res = subprocess.run(
                        ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", out_prefix],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True
                    )
                    if res.returncode == 0:
                        self.current_mode = "NAV_READY"
                        self.ui(f"✅ Đã lưu map: {out_prefix}.yaml / {out_prefix}.pgm")
                        self.get_logger().info("MODE → NAV_READY")

                        # Pre-warm navigation so "Bắt đầu đi tới B" phản ứng ngay (không phải chờ launch).
                        # grid_nav will idle until it receives /goal_pose.
                        self.start_proc("goto_goal", ["ros2", "launch", "auto_explore", "goto_goal.launch.py"])
                    else:
                        self.current_mode = "AUTO_EXPLORE"
                        self.ui("❌ Lưu map thất bại. Xem log.")
                        self.get_logger().error(res.stderr[-800:] if res.stderr else "map_saver_cli failed")
                except Exception as e:
                    self.current_mode = "AUTO_EXPLORE"
                    self.ui(f"❌ Lưu map lỗi: {e}")

            threading.Thread(target=_save_job, daemon=True).start()
            return

        self.get_logger().warn(f"Unknown explore_cmd: {cmd}")

    # =====================================================
    # GOTO GOAL HANDLER
    # =====================================================
    def on_goto_cmd(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"[GOTO_CMD] {cmd}")

        if cmd == "START":
            if self.current_mode == "SAVING_MAP":
                self.ui("⏳ Đang lưu map, vui lòng đợi lưu xong rồi hãy đi tới B.")
                return

            # Ensure NAV_READY: stop explore to avoid cmd_vel conflicts
            if self.current_mode != "NAV_READY":
                self.ui("Chuẩn bị điều hướng: dừng Explore và chuyển sang NAV_READY...")
                self.stop_process_by_name("explore")
                self.send_stop_signal()
                self.current_mode = "NAV_READY"
                self.ui("✅ NAV_READY. Bắt đầu đi tới điểm B...")

            # If navigation was pre-warmed at NAV_READY, don't relaunch (avoid delay and duplicates).
            if not any(getattr(p, "_name", "") == "goto_goal" and p.poll() is None for p in self.procs):
                self.start_proc("goto_goal", ["ros2", "launch", "auto_explore", "goto_goal.launch.py"])
            self.ui("🚗 Đang điều hướng tới điểm B...")
            return

        if cmd == "STOP":
            self.stop_process_by_name("goto_goal")
            self.send_stop_signal()
            self.ui("Đã dừng điều hướng.")
            return

        self.get_logger().warn(f"Unknown goto_goal cmd: {cmd}")


def main():
    rclpy.init()
    node = ModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.kill_all_child()
        node.kill_cameras()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
