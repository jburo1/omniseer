import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_symlinked_omni_resolves_repo_scripts(tmp_path: Path) -> None:
    launcher = tmp_path / "omni"
    launcher.symlink_to(REPO_ROOT / "scripts" / "omni")

    result = subprocess.run(
        [str(launcher), "env"],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "repo_root=" in result.stdout
