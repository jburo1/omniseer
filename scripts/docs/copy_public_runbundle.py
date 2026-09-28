"""Add the public target-acquisition RunBundle to every MkDocs build."""

import shutil
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RUNBUNDLE_SOURCE = REPOSITORY_ROOT / "studies/autonomy/v2l_fp_target_acquisition/run"
RUNBUNDLE_DESTINATION = Path("runs/v2l_fp_scene_1")
CANONICAL_REPORT_SOURCE = RUNBUNDLE_SOURCE / "report-updated"


def on_post_build(config, **kwargs):
    """Publish the complete RunBundle with its current report at a stable URL."""
    destination = Path(config["site_dir"]) / RUNBUNDLE_DESTINATION
    shutil.rmtree(destination, ignore_errors=True)
    shutil.copytree(RUNBUNDLE_SOURCE, destination)

    # Preserve the original report derivative in the evidence bundle, but
    # publish the corrected/current presentation at the long-lived /report/
    # URL used by the documentation and external links.
    shutil.rmtree(destination / "report")
    shutil.copytree(CANONICAL_REPORT_SOURCE, destination / "report")
