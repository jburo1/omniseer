import json
import tempfile
import unittest
from pathlib import Path

from omniseer_experiments.commissioning_analysis import _storage_id, _write_plots, analyze_samples


def _twist(time: float, vx: float, vy: float, wz: float) -> dict:
    return {"time": time, "vx": vx, "vy": vy, "wz": wz}


def _odom(time: float, vx: float, vy: float, wz: float, x: float, y: float, yaw: float) -> dict:
    return {**_twist(time, vx, vy, wz), "x": x, "y": y, "yaw": yaw}


class CommissioningAnalysisTest(unittest.TestCase):
    def test_segments_on_phase_transitions_and_distinguishes_measurements(self) -> None:
        samples = {
            "/commissioning/phase": [
                {"time": 0.0, "name": "settle"},
                {"time": 0.2, "name": "settle"},
                {"time": 1.0, "name": "forward"},
                {"time": 2.0, "name": "settle_final"},
            ],
            "/cmd_vel_autonomy": [_twist(0.5, 0.0, 0.0, 0.0), _twist(1.1, 0.1, 0.0, 0.0), _twist(1.8, 0.1, 0.0, 0.0), _twist(2.1, 0.0, 0.0, 0.0)],
            "/mecanum_drive_controller/reference": [_twist(1.1, 0.1, 0.0, 0.0), _twist(1.8, 0.1, 0.0, 0.0)],
            "/mecanum_drive_controller/odometry": [_odom(0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), _odom(1.1, 0.02, 0.01, 0.0, 0.01, 0.001, 0.0), _odom(1.5, 0.1, 0.01, 0.0, 0.05, 0.004, 0.0), _odom(1.8, 0.1, 0.01, 0.0, 0.08, 0.007, 0.0), _odom(2.1, 0.0, 0.0, 0.0, 0.08, 0.007, 0.0)],
            "/odometry/filtered": [_odom(1.1, 0.01, 0.0, 0.0, 0.01, 0.0, 0.0), _odom(1.5, 0.08, 0.0, 0.0, 0.04, 0.0, 0.0), _odom(1.8, 0.09, 0.0, 0.0, 0.07, 0.0, 0.0), _odom(2.1, 0.0, 0.0, 0.0, 0.07, 0.0, 0.0)],
            "/imu": [{"time": 0.5, "wz": 0.02}, {"time": 1.1, "wz": 0.01}, {"time": 1.8, "wz": 0.01}, {"time": 2.1, "wz": 0.02}],
            "/encoder_counts": [{"time": 1.1, "front_left": 10, "front_right": 11, "rear_left": 10, "rear_right": 11}, {"time": 1.8, "front_left": 20, "front_right": 21, "rear_left": 20, "rear_right": 21}],
        }

        summary = analyze_samples(samples)

        self.assertEqual(summary["phase_count"], 3)
        forward = summary["phases"][1]
        self.assertEqual(forward["name"], "forward")
        self.assertAlmostEqual(forward["duration_sec"], 1.0)
        self.assertAlmostEqual(forward["raw_wheel_odometry_velocity"]["vx"]["mean"], 0.07333333333333333)
        self.assertAlmostEqual(
            forward["cross_axis_response"]["raw_wheel_odometry"]["other_axes"]["vy"]["mean"], 0.01
        )
        self.assertEqual(forward["encoder_count_change"]["front_left"], 10.0)
        self.assertAlmostEqual(forward["integrated_raw_odometry_pose_change"]["x_m"], 0.07)
        self.assertIn("no external position", summary["interpretation"]["odometry_and_imu"])

    def test_writes_svg_plots_without_optional_plotting_dependencies(self) -> None:
        phases = [{"name": "settle", "start_sec": 0.0, "end_sec": 1.0}]
        samples = {
            "/cmd_vel_autonomy": [_twist(0.0, 0.0, 0.0, 0.0)],
            "/mecanum_drive_controller/reference": [_twist(0.0, 0.0, 0.0, 0.0)],
            "/mecanum_drive_controller/odometry": [_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)],
            "/odometry/filtered": [_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)],
            "/imu": [{"time": 0.0, "wz": 0.0}],
        }
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            _write_plots(output, samples, phases)
            self.assertEqual({path.name for path in output.iterdir()}, {"vx.svg", "vy.svg", "wz.svg", "trajectory_xy.svg", "yaw.svg"})
            self.assertIn("phase-label", (output / "vx.svg").read_text(encoding="utf-8"))

    def test_summary_is_json_serializable(self) -> None:
        summary = analyze_samples({"/commissioning/phase": [{"time": 0.0, "name": "settle"}], "/cmd_vel_autonomy": [_twist(0.1, 0, 0, 0)]})
        self.assertIn("phase_count", json.loads(json.dumps(summary)))

    def test_selects_mcap_storage_from_rosbag_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            bag_dir = Path(directory)
            (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  storage_identifier: mcap\n")
            self.assertEqual(_storage_id(bag_dir), "mcap")
