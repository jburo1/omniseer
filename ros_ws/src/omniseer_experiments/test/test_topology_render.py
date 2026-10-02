import json
import shutil
import tempfile
import unittest
from pathlib import Path

from omniseer_experiments.topology_render import (
    bounded_autonomy_topology_to_dot,
    render_topology,
    topology_to_dot,
)


class TopologyRenderTests(unittest.TestCase):
    def test_filters_runtime_noise_and_retains_observed_one_sided_endpoints(self) -> None:
        topology = {
            "nodes": [
                {"name": "vision_bridge", "namespace": "/"},
                {"name": "perception_run_recorder", "namespace": "/"},
                {"name": "omniseer_teensy", "namespace": "/"},
                {"name": "rosbag2_recorder", "namespace": "/"},
                {"name": "transform_listener_impl_123", "namespace": "/"},
            ],
            "topics": [
                {
                    "name": "/yolo/detections",
                    "publishers": [{"node_name": "vision_bridge", "node_namespace": "/"}],
                    "subscribers": [{"node_name": "perception_run_recorder", "node_namespace": "/"}],
                },
                {
                    "name": "/one_sided",
                    "publishers": [],
                    "subscribers": [{"node_name": "observed_subscriber", "node_namespace": "/"}],
                },
                {
                    "name": "/rosout",
                    "publishers": [{"node_name": "vision_bridge", "node_namespace": "/"}],
                    "subscribers": [],
                },
                {
                    "name": "/battery",
                    "publishers": [],
                    "subscribers": [{"node_name": "perception_run_recorder", "node_namespace": "/"}],
                },
                {
                    "name": "/set_pose",
                    "publishers": [],
                    "subscribers": [{"node_name": "vision_bridge", "node_namespace": "/"}],
                },
                {
                    "name": "/tf",
                    "publishers": [{"node_name": "vision_bridge", "node_namespace": "/"}],
                    "subscribers": [],
                },
                {
                    "name": "/cmd_vel_nav",
                    "publishers": [],
                    "subscribers": [{"node_name": "observed_subscriber", "node_namespace": "/"}],
                },
                {
                    "name": "/self",
                    "publishers": [{"node_name": "vision_bridge", "node_namespace": "/"}],
                    "subscribers": [{"node_name": "vision_bridge", "node_namespace": "/"}],
                },
            ],
        }
        topology["topics"].extend(
            {
                "name": topic_name,
                "publishers": [{"node_name": "vision_bridge", "node_namespace": "/"}],
                "subscribers": [],
            }
            for topic_name in ("/diagnostics", "/events/write_split", "/parameter_events", "/tf_static")
        )

        dot = topology_to_dot(topology)

        self.assertIn('label="/vision_bridge"', dot)
        self.assertIn('label="/perception_run_recorder"', dot)
        self.assertIn('label="/yolo/detections"', dot)
        self.assertIn('label="/one_sided"', dot)
        self.assertIn('label="/observed_subscriber"', dot)
        self.assertIn('label="/cmd_vel_nav"', dot)
        self.assertIn('label="/battery"', dot)
        self.assertIn('label="/set_pose"', dot)
        self.assertNotIn("/rosout", dot)
        for ignored in (
            "/diagnostics",
            "/events/write_split",
            "/parameter_events",
            "/tf",
            "/tf_static",
        ):
            self.assertNotIn(ignored, dot)
        self.assertIn('label="/rosbag2_recorder"', dot)
        self.assertNotIn("transform_listener_impl_123", dot)
        self.assertNotIn("subgraph cluster_", dot)

    @unittest.skipUnless(shutil.which("dot"), "Graphviz dot is required")
    def test_writes_dot_and_svg_without_changing_input(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            input_path = Path(tmp) / "topology.json"
            output_path = Path(tmp) / "topology.svg"
            input_path.write_text(
                json.dumps({"nodes": [], "topics": [{"name": "/one_sided", "publishers": [], "subscribers": []}]}),
                encoding="utf-8",
            )
            before = input_path.read_bytes()

            dot_path = render_topology(input_path, output_path)

            self.assertEqual(input_path.read_bytes(), before)
            self.assertTrue(dot_path.is_file())
            self.assertTrue(output_path.is_file())
            self.assertIn("<svg", output_path.read_text(encoding="utf-8"))

    def test_bounded_autonomy_projection_retains_only_observed_path_endpoints(self) -> None:
        topology = {
            "nodes": [{"name": "omniseer_teensy", "namespace": "/"}],
            "topics": [
                {
                    "name": "/yolo/detections",
                    "publishers": [{"node_name": "vision_bridge", "node_namespace": "/"}],
                    "subscribers": [
                        {"node_name": "target_centering_node", "node_namespace": "/"},
                        {"node_name": "perception_run_recorder", "node_namespace": "/"},
                    ],
                },
                {
                    "name": "/cmd_vel_autonomy",
                    "publishers": [{"node_name": "target_centering_node", "node_namespace": "/"}],
                    "subscribers": [{"node_name": "twist_mux", "node_namespace": "/"}],
                },
                {
                    "name": "/encoder_counts",
                    "publishers": [],
                    "subscribers": [{"node_name": "encoder_counts_to_odometry", "node_namespace": "/"}],
                },
                {
                    "name": "/scan",
                    "publishers": [{"node_name": "rplidar_composition", "node_namespace": "/"}],
                    "subscribers": [{"node_name": "rosbag2_recorder", "node_namespace": "/"}],
                },
            ],
        }

        dot = bounded_autonomy_topology_to_dot(topology)

        for group in (
            "Hardware / I/O",
            "Perception",
            "State Estimation",
            "Autonomy / Control",
            "Run Evidence",
        ):
            self.assertIn(f'label="{group}"', dot)
        self.assertIn('label="/vision_bridge"', dot)
        self.assertIn('label="/target_centering_node"', dot)
        self.assertIn('label="/encoder_counts"', dot)
        self.assertIn('label="/encoder_counts_to_odometry"', dot)
        self.assertIn("style=dashed", dot)
        self.assertIn("style=dotted", dot)
        self.assertIn('label="/omniseer_teensy"', dot)
        self.assertNotIn("robot_diag_control_cpp", dot)
        self.assertNotIn("rplidar_composition", dot)
        self.assertNotIn('label="/scan"', dot)
