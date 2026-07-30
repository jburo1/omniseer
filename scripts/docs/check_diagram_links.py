#!/usr/bin/env python3
"""Validate internal links embedded in built SVG diagram assets."""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote, urlparse

IGNORED_SCHEMES = {"http", "https", "mailto", "tel", "data"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate internal href values in built SVG diagram assets.",
    )
    parser.add_argument(
        "--site-dir",
        default="site",
        type=Path,
        help="MkDocs build output directory. Defaults to site.",
    )
    return parser.parse_args()


def iter_svg_hrefs(svg_path: Path) -> set[str]:
    tree = ET.parse(svg_path)
    hrefs: set[str] = set()

    for element in tree.iter():
        for key, value in element.attrib.items():
            if key == "href" or key.endswith("}href"):
                hrefs.add(value)

    return hrefs


def should_ignore_href(href: str) -> bool:
    parsed = urlparse(href)
    return not href or href.startswith("#") or parsed.scheme in IGNORED_SCHEMES or bool(parsed.netloc)


def built_target_exists(target_path: Path) -> bool:
    if target_path.is_dir():
        return (target_path / "index.html").is_file()

    if target_path.is_file():
        return True

    if target_path.suffix:
        return False

    return (target_path / "index.html").is_file()


def validate_href(site_dir: Path, svg_path: Path, href: str) -> str | None:
    if should_ignore_href(href):
        return None

    parsed = urlparse(href)
    if not parsed.path:
        return None

    target_path = (svg_path.parent / unquote(parsed.path)).resolve()
    try:
        target_path.relative_to(site_dir)
    except ValueError:
        return f"{svg_path}: link escapes site directory: {href}"

    if built_target_exists(target_path):
        return None

    return f"{svg_path}: missing internal target for {href} -> {target_path}"


def main() -> int:
    args = parse_args()
    site_dir = args.site_dir.resolve()
    diagram_dir = site_dir / "assets" / "diagrams"

    if not site_dir.is_dir():
        print(f"error: site directory does not exist: {site_dir}", file=sys.stderr)
        return 2

    if not diagram_dir.is_dir():
        print(f"error: built diagram directory does not exist: {diagram_dir}", file=sys.stderr)
        return 2

    failures: list[str] = []
    checked_hrefs = 0
    svg_paths = sorted(diagram_dir.rglob("*.svg"))

    if not svg_paths:
        print(f"error: no built SVG diagrams found under {diagram_dir}", file=sys.stderr)
        return 2

    for svg_path in svg_paths:
        try:
            hrefs = iter_svg_hrefs(svg_path)
        except ET.ParseError as exc:
            failures.append(f"{svg_path}: invalid SVG XML: {exc}")
            continue

        for href in sorted(hrefs):
            failure = validate_href(site_dir, svg_path.resolve(), href)
            if failure is not None:
                failures.append(failure)
            elif not should_ignore_href(href) and urlparse(href).path:
                checked_hrefs += 1

    if failures:
        for failure in failures:
            print(f"error: {failure}", file=sys.stderr)
        return 2

    print(f"info: checked {checked_hrefs} internal SVG diagram links")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
