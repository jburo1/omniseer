import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts/docs/check_site_links.py"


def _run_validator(site_dir: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--site-dir", str(site_dir)],
        capture_output=True,
        text=True,
        check=False,
    )


def test_generated_site_validator_resolves_urls_from_the_page_location(tmp_path: Path) -> None:
    site_dir = tmp_path / "site"
    page_path = site_dir / "architecture/index.html"
    page_path.parent.mkdir(parents=True)
    (site_dir / "assets").mkdir()
    (site_dir / "assets/system.svg").touch()
    (site_dir / "index.html").touch()
    page_path.write_text(
        """<object data=\"../assets/system.svg\"></object>
        <a href=\"../index.html#top\">Home</a>
        <img src=\"https://example.test/image.svg\">
        <a href=\"mailto:ops@example.test\">Contact</a>
        <a href=\"#section\">Section</a>""",
        encoding="utf-8",
    )

    result = _run_validator(site_dir)

    assert result.returncode == 0, result.stderr
    assert "checked 2 local HTML URLs" in result.stdout


def test_generated_site_validator_reports_page_url_and_missing_target(tmp_path: Path) -> None:
    site_dir = tmp_path / "site"
    page_path = site_dir / "architecture/index.html"
    page_path.parent.mkdir(parents=True)
    page_path.write_text('<object data="../assets/missing.svg"></object>', encoding="utf-8")

    result = _run_validator(site_dir)

    assert result.returncode == 2
    assert str(page_path.resolve()) in result.stderr
    assert "../assets/missing.svg" in result.stderr
    assert str((site_dir / "assets/missing.svg").resolve()) in result.stderr


def test_generated_site_validator_maps_absolute_urls_from_mkdocs_deployment_path(tmp_path: Path) -> None:
    site_dir = tmp_path / "site"
    site_dir.mkdir()
    (site_dir / "index.html").write_text(
        '<link rel="canonical" href="https://example.test/omniseer/">', encoding="utf-8"
    )
    (site_dir / "assets").mkdir()
    (site_dir / "assets/logo.svg").touch()

    result = _run_validator(site_dir)
    assert result.returncode == 0, result.stderr

    (site_dir / "index.html").write_text(
        '<link rel="canonical" href="https://example.test/omniseer/"><img src="/omniseer/assets/logo.svg">',
        encoding="utf-8",
    )
    result = _run_validator(site_dir)

    assert result.returncode == 0, result.stderr

    (site_dir / "index.html").write_text(
        '<link rel="canonical" href="https://example.test/omniseer/"><img src="/assets/logo.svg">',
        encoding="utf-8",
    )
    result = _run_validator(site_dir)

    assert result.returncode == 2
    assert "resolves outside the site" in result.stderr


def test_generated_site_validator_rejects_targets_outside_site(tmp_path: Path) -> None:
    site_dir = tmp_path / "site"
    site_dir.mkdir()
    (site_dir / "index.html").write_text('<img src="../outside.svg">', encoding="utf-8")
    (tmp_path / "outside.svg").touch()

    result = _run_validator(site_dir)

    assert result.returncode == 2
    assert "resolves outside the site" in result.stderr
