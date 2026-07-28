from __future__ import annotations

import math
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def pose_from_mapping(navigator: BasicNavigator, values: dict[str, Any]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = str(values.get("frame_id", "map"))
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(values["x"])
    pose.pose.position.y = float(values["y"])
    pose.pose.position.z = float(values.get("z", 0.0))

    qx, qy, qz, qw = quaternion_from_yaw(float(values.get("yaw", 0.0)))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class MissionError(RuntimeError):
    pass


class InterviewMission:
    def __init__(self, navigator: BasicNavigator, script_path: Path, audio_enabled: bool) -> None:
        self.navigator = navigator
        self.script_path = script_path
        self.audio_enabled = audio_enabled
        self.package_share = Path(get_package_share_directory("socialbot_demo0"))
        self.audio_dir = self.package_share / "assets" / "audio"

        with script_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)

        if not isinstance(data, dict):
            raise MissionError(f"Invalid mission file: {script_path}")

        self.initial_pose = data.get("initial_pose")
        self.targets = data.get("targets", {})
        self.steps = data.get("steps", [])

        if not isinstance(self.initial_pose, dict):
            raise MissionError("The mission must contain an initial_pose mapping")
        if not isinstance(self.targets, dict):
            raise MissionError("The mission targets field must be a mapping")
        if not isinstance(self.steps, list):
            raise MissionError("The mission steps field must be a list")

    def wait_for_clock(self, timeout: float = 30.0) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.navigator.get_clock().now().nanoseconds == 0:
            if time.monotonic() >= deadline:
                raise MissionError("Simulation clock did not start")
            rclpy.spin_once(self.navigator, timeout_sec=0.1)

    def initialise_navigation(self) -> None:
        self.wait_for_clock()
        initial = pose_from_mapping(self.navigator, self.initial_pose)
        self.navigator.setInitialPose(initial)
        self.navigator.get_logger().info("Waiting for Nav2 and AMCL to become active")
        self.navigator.waitUntilNav2Active(localizer="amcl")
        self.navigator.get_logger().info("Nav2 is active; starting Demo 0 mission")

    def play_audio(self, filename: str) -> None:
        if not self.audio_enabled:
            self.navigator.get_logger().info(f"Audio disabled; would play {filename}")
            return

        audio_path = self.audio_dir / filename
        if not audio_path.is_file():
            raise MissionError(f"Audio file not found: {audio_path}")
        if shutil.which("paplay") is None:
            raise MissionError("paplay is not installed in the container")

        self.navigator.get_logger().info(f"Playing {filename}")
        result = subprocess.run(
            ["paplay", str(audio_path)],
            check=False,
            timeout=120.0,
        )
        if result.returncode != 0:
            raise MissionError(f"paplay failed for {filename} with code {result.returncode}")

    def navigate(self, target_name: str, timeout: float) -> None:
        if target_name not in self.targets:
            raise MissionError(f"Unknown target: {target_name}")

        target = pose_from_mapping(self.navigator, self.targets[target_name])
        target.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.get_logger().info(f"Navigating to {target_name}")
        self.navigator.goToPose(target)

        started = time.monotonic()
        while rclpy.ok() and not self.navigator.isTaskComplete():
            if time.monotonic() - started > timeout:
                self.navigator.cancelTask()
                raise MissionError(f"Navigation to {target_name} timed out after {timeout:.1f} s")
            time.sleep(0.1)

        result = self.navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            raise MissionError(f"Navigation to {target_name} failed with result {result}")

        self.navigator.get_logger().info(f"Reached {target_name}")

    def run(self) -> None:
        self.initialise_navigation()

        for index, step in enumerate(self.steps, start=1):
            if not isinstance(step, dict):
                raise MissionError(f"Step {index} must be a mapping")

            action = str(step.get("action", "")).strip().lower()
            if action == "navigate":
                self.navigate(
                    target_name=str(step["target"]),
                    timeout=float(step.get("timeout", 120.0)),
                )
            elif action == "audio":
                self.play_audio(str(step["file"]))
            elif action == "wait":
                seconds = max(0.0, float(step.get("seconds", 1.0)))
                self.navigator.get_logger().info(f"Waiting {seconds:.1f} s")
                time.sleep(seconds)
            else:
                raise MissionError(f"Unsupported action in step {index}: {action!r}")

        self.navigator.get_logger().info("Demo 0 mission completed")


def main(args=None) -> None:
    rclpy.init(args=args)
    navigator = BasicNavigator()

    package_share = Path(get_package_share_directory("socialbot_demo0"))
    default_script = package_share / "config" / "interview_script.yaml"

    navigator.declare_parameter("script_file", str(default_script))
    navigator.declare_parameter("audio_enabled", True)

    script_path = Path(str(navigator.get_parameter("script_file").value)).expanduser()
    audio_enabled = bool(navigator.get_parameter("audio_enabled").value)

    try:
        mission = InterviewMission(navigator, script_path, audio_enabled)
        mission.run()
    except KeyboardInterrupt:
        navigator.get_logger().warning("Mission interrupted by the operator")
        navigator.cancelTask()
    except (MissionError, OSError, subprocess.SubprocessError, yaml.YAMLError) as exc:
        navigator.get_logger().error(f"Demo 0 failed: {exc}")
        navigator.cancelTask()
        try:
            fallback = package_share / "assets" / "audio" / "navigation_failed.wav"
            if audio_enabled and fallback.is_file() and shutil.which("paplay"):
                subprocess.run(["paplay", str(fallback)], check=False, timeout=30.0)
        except Exception:  # noqa: BLE001 - fallback must never hide the original error.
            pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
