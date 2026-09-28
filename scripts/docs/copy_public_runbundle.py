"""Add the public target-acquisition RunBundle to every MkDocs build."""

import shutil
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RUNBUNDLE_SOURCE = REPOSITORY_ROOT / "studies/autonomy/v2l_fp_target_acquisition/run"
RUNBUNDLE_DESTINATION = Path("runs/v2l_fp_scene_1")


def on_post_build(config, **kwargs):
    """Copy the complete RunBundle after MkDocs has populated ``site_dir``."""
    destination = Path(config["site_dir"]) / RUNBUNDLE_DESTINATION
    shutil.rmtree(destination, ignore_errors=True)
    shutil.copytree(RUNBUNDLE_SOURCE, destination)
