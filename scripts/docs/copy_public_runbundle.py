"""Publish the public target-acquisition RunBundle without duplicating LFS media."""

import posixpath
import re
import shutil
import subprocess
from pathlib import Path, PurePosixPath
from urllib.parse import urlsplit, urlunsplit

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RUNBUNDLE_SOURCE = REPOSITORY_ROOT / "studies/autonomy/v2l_fp_target_acquisition/run"
RUNBUNDLE_DESTINATION = Path("runs/v2l_fp_scene_1")
CANONICAL_REPORT_SOURCE = RUNBUNDLE_SOURCE / "report-updated"
CHECKSUMS_SOURCE = RUNBUNDLE_SOURCE.parent / "checksums.sha256"
MEDIA_SUFFIXES = {".jpg", ".jpeg", ".mcap", ".mp4", ".png", ".ts", ".webp"}
LFS_POINTER_PREFIX = b"version https://git-lfs.github.com/spec/v1"
LINK_ATTRIBUTE_PATTERN = re.compile(r'(?P<prefix>\b(?:href|src)\s*=\s*["\'])(?P<url>[^"\']+)', re.IGNORECASE)


def lfs_managed_paths():
    """Return repository-relative files selected by Git LFS attributes."""
    paths = [path.relative_to(REPOSITORY_ROOT) for path in RUNBUNDLE_SOURCE.rglob("*") if path.is_file()]
    result = subprocess.run(
        ["git", "check-attr", "-z", "--stdin", "filter"],
        cwd=REPOSITORY_ROOT,
        input="\0".join(path.as_posix() for path in paths) + "\0",
        capture_output=True,
        check=True,
        text=True,
    )
    fields = result.stdout.split("\0")
    return {
        Path(path)
        for path, attribute, value in zip(fields[0::3], fields[1::3], fields[2::3])
        if attribute == "filter" and value == "lfs"
    }


def copy_without_lfs(destination, lfs_paths):
    """Copy textual RunBundle artifacts, leaving LFS objects in GitHub's media store."""

    def ignore(directory, names):
        relative_directory = Path(directory).relative_to(REPOSITORY_ROOT)
        return [name for name in names if relative_directory / name in lfs_paths]

    shutil.copytree(RUNBUNDLE_SOURCE, destination, ignore=ignore)


def rewrite_lfs_links(report_source, report_destination, lfs_paths):
    """Point report media at GitHub's LFS-aware media endpoint after publication."""
    report_directory = report_source.relative_to(REPOSITORY_ROOT).parent

    def replace(match):
        parsed = urlsplit(match.group("url"))
        if parsed.scheme or parsed.netloc or parsed.path.startswith("/"):
            return match.group(0)
        target = PurePosixPath(posixpath.normpath((report_directory / PurePosixPath(parsed.path)).as_posix()))
        if Path(target) not in lfs_paths:
            return match.group(0)
        url = urlunsplit(
            (
                "https",
                "media.githubusercontent.com",
                "/media/jburo1/omniseer/master/" + target.as_posix(),
                parsed.query,
                parsed.fragment,
            )
        )
        return match.group("prefix") + url

    report_destination.write_text(LINK_ATTRIBUTE_PATTERN.sub(replace, report_source.read_text()), encoding="utf-8")


def validate_no_lfs_media(destination):
    """Fail a docs build if Pages would serve an LFS pointer as a media artifact."""
    pointers = [
        path.relative_to(destination)
        for path in destination.rglob("*")
        if path.is_file() and path.suffix.lower() in MEDIA_SUFFIXES and path.read_bytes().startswith(LFS_POINTER_PREFIX)
    ]
    if pointers:
        formatted_paths = ", ".join(str(path) for path in pointers)
        raise RuntimeError(f"Git LFS pointers would be published as media: {formatted_paths}")
    print("info: checked published media for Git LFS pointers")


def on_post_build(config, **kwargs):
    """Publish the lightweight bundle and a report whose media resolves from GitHub."""
    destination = Path(config["site_dir"]) / RUNBUNDLE_DESTINATION
    shutil.rmtree(destination, ignore_errors=True)
    lfs_paths = lfs_managed_paths()
    copy_without_lfs(destination, lfs_paths)
    shutil.copy2(CHECKSUMS_SOURCE, destination / CHECKSUMS_SOURCE.name)

    # Preserve the original report derivative in the evidence bundle, but
    # publish the corrected/current presentation at the stable /report/ URL.
    shutil.rmtree(destination / "report")
    shutil.copytree(CANONICAL_REPORT_SOURCE, destination / "report")

    rewrite_lfs_links(
        RUNBUNDLE_SOURCE / "report-updated/index.html",
        destination / "report-updated/index.html",
        lfs_paths,
    )
    rewrite_lfs_links(
        CANONICAL_REPORT_SOURCE / "index.html",
        destination / "report/index.html",
        lfs_paths,
    )
    validate_no_lfs_media(Path(config["site_dir"]))
