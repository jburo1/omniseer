#!/usr/bin/env python3
"""Validate local URL-bearing attributes in generated HTML pages.

This intentionally checks only the generated site's HTML.  SVG-internal links
and LFS media are validated by their dedicated checks in the docs build flow.
"""

from __future__ import annotations

import argparse
import sys
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit

URL_ATTRIBUTES = {"href", "src", "data"}


class UrlAttributeExtractor(HTMLParser):
    """Collect URL-bearing attributes without making network requests."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.urls: list[tuple[str, str]] = []
        self.canonical_url: str | None = None

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self._collect(tag, attrs)

    def handle_startendtag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self._collect(tag, attrs)

    def _collect(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attributes = {name.lower(): value for name, value in attrs}
        if tag.lower() == "link" and attributes.get("rel") == "canonical":
            self.canonical_url = attributes.get("href")
        for name, value in attrs:
            if name.lower() in URL_ATTRIBUTES and value is not None:
                self.urls.append((name.lower(), value))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate local href, src, and data URLs in generated HTML pages.",
    )
    parser.add_argument(
        "--site-dir",
        default="site",
        type=Path,
        help="MkDocs build output directory. Defaults to site.",
    )
    return parser.parse_args()


def is_local_url(url: str) -> bool:
    """Return whether *url* denotes a local, non-anchor resource."""
    parsed = urlsplit(url)
    return bool(url) and not url.startswith("#") and not parsed.scheme and not parsed.netloc


def resolve_local_url(site_dir: Path, page_path: Path, url: str, site_url_path: str) -> Path:
    """Resolve a local URL exactly from the generated page's filesystem location."""
    parsed = urlsplit(url)
    decoded_path = unquote(parsed.path)
    if decoded_path.startswith("/"):
        if site_url_path != "/" and not decoded_path.startswith(site_url_path):
            return (site_dir.parent / decoded_path.lstrip("/")).resolve()
        return (site_dir / decoded_path.removeprefix(site_url_path).lstrip("/")).resolve()
    return (page_path.parent / decoded_path).resolve()


def validate_url(
    site_dir: Path,
    page_path: Path,
    attribute: str,
    url: str,
    site_url_path: str,
) -> str | None:
    if not is_local_url(url):
        return None

    target_path = resolve_local_url(site_dir, page_path, url, site_url_path)
    try:
        target_path.relative_to(site_dir)
    except ValueError:
        return f"{page_path}: local {attribute} URL {url!r} resolves outside the site: {target_path}"

    if target_path.exists():
        return None

    return f"{page_path}: missing local {attribute} URL {url!r} -> {target_path}"


def site_url_path(site_dir: Path) -> str:
    """Infer the deployment path from MkDocs' canonical homepage URL."""
    index_path = site_dir / "index.html"
    if not index_path.is_file():
        return "/"
    parser = UrlAttributeExtractor()
    parser.feed(index_path.read_text(encoding="utf-8"))
    parser.close()
    canonical_path = urlsplit(parser.canonical_url or "").path
    return canonical_path if canonical_path.endswith("/") else f"{canonical_path}/"


def validate_site_links(site_dir: Path) -> tuple[int, list[str]]:
    """Return the number of checked URLs and any missing-target failures."""
    checked_urls = 0
    failures: list[str] = []
    deployment_path = site_url_path(site_dir)
    for page_path in sorted(site_dir.rglob("*.html")):
        parser = UrlAttributeExtractor()
        parser.feed(page_path.read_text(encoding="utf-8"))
        parser.close()
        for attribute, url in parser.urls:
            if not is_local_url(url):
                continue
            checked_urls += 1
            failure = validate_url(site_dir, page_path.resolve(), attribute, url, deployment_path)
            if failure is not None:
                failures.append(failure)
    return checked_urls, failures


def main() -> int:
    args = parse_args()
    site_dir = args.site_dir.resolve()
    if not site_dir.is_dir():
        print(f"error: site directory does not exist: {site_dir}", file=sys.stderr)
        return 2

    html_paths = list(site_dir.rglob("*.html"))
    if not html_paths:
        print(f"error: no generated HTML pages found under {site_dir}", file=sys.stderr)
        return 2

    checked_urls, failures = validate_site_links(site_dir)
    if failures:
        for failure in failures:
            print(f"error: {failure}", file=sys.stderr)
        return 2

    print(f"info: checked {checked_urls} local HTML URLs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
