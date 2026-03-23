import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


@unittest.skipUnless(
    os.environ.get("OMNISEER_RUN_SIM_SMOKE") == "1",
    "set OMNISEER_RUN_SIM_SMOKE=1 to run the headless Gazebo smoke test",
)
class SimLaunchSmokeTests(unittest.TestCase):
    def setUp(self) -> None:
        smoke_log_path = os.environ.get("OMNISEER_SMOKE_LOG_PATH")
        if smoke_log_path is None:
            log_fd, generated_log_path = tempfile.mkstemp(prefix="omniseer-sim-", suffix=".log")
            os.close(log_fd)
            smoke_log_path = generated_log_path

        self._launch_log_path = Path(smoke_log_path)
        self._launch_log_path.parent.mkdir(parents=True, exist_ok=True)
        self._launch_log = self._launch_log_path.open("w", encoding="utf-8")
        self._launch_process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "bringup",
                "sim.launch.py",
                "world:=ci_smoke.world",
                "use_ci_geometry:=true",
                "headless:=true",
                "log_level:=warn",
                "start_nav:=false",
                "start_slam:=false",
                "start_rf2o:=false",
                "start_rviz:=false",
                "start_gateway:=false",
            ],
            cwd=_repo_root(),
            stdout=self._launch_log,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )

    def tearDown(self) -> None:
        if self._launch_process.poll() is None:
            os.killpg(self._launch_process.pid, signal.SIGINT)
            try:
                self._launch_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(self._launch_process.pid, signal.SIGTERM)
                try:
                    self._launch_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(self._launch_process.pid, signal.SIGKILL)
                    self._launch_process.wait(timeout=5.0)

        self._launch_log.close()

    def test_headless_sim_launch_exposes_boundary_topics(self) -> None:
        expected_topics = {
            "/clock": "rosgraph_msgs/msg/Clock",
            "/imu": "sensor_msgs/msg/Imu",
            "/scan": "sensor_msgs/msg/LaserScan",
            "/range": "sensor_msgs/msg/Range",
            "/mecanum_drive_controller/odometry": "nav_msgs/msg/Odometry",
        }

        for topic_name, topic_type in expected_topics.items():
            self._wait_for_topic_type(topic_name, topic_type, timeout_seconds=120.0)

        self._assert_launch_is_still_running()

    def _wait_for_topic_type(
        self,
        topic_name: str,
        expected_type: str,
        *,
        timeout_seconds: float,
    ) -> None:
        deadline = time.monotonic() + timeout_seconds
        last_seen_type = ""

        while time.monotonic() < deadline:
            self._assert_launch_is_still_running()

            result = subprocess.run(
                ["ros2", "topic", "type", topic_name],
                check=False,
                capture_output=True,
                text=True,
                timeout=10.0,
            )
            last_seen_type = result.stdout.strip()
            if result.returncode == 0 and last_seen_type == expected_type:
                return

            time.sleep(1.0)

        self.fail(
            f"Timed out waiting for {topic_name} to report type {expected_type}; "
            f"last_seen_type={last_seen_type!r}\n\nlaunch log tail:\n{self._launch_log_tail()}"
        )

    def _assert_launch_is_still_running(self) -> None:
        return_code = self._launch_process.poll()
        if return_code is None:
            return

        self.fail(
            f"ros2 launch exited unexpectedly with return code {return_code}\n\n"
            f"launch log tail:\n{self._launch_log_tail()}"
        )

    def _launch_log_tail(self, *, line_count: int = 120) -> str:
        try:
            lines = self._launch_log_path.read_text(encoding="utf-8").splitlines()
        except FileNotFoundError:
            return "<launch log unavailable>"
        return "\n".join(lines[-line_count:])
