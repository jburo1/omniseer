#!/usr/bin/env python3
"""Installed-script wrapper for the real bringup topic readiness gate."""

from __future__ import annotations

import sys
from pathlib import Path


if __package__ in {None, ""}:
    module_path = Path(__file__).resolve().parents[1] / "bringup" / "wait_for_topics.py"
    import importlib.util

    spec = importlib.util.spec_from_file_location("bringup_wait_for_topics_source", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    main = module.main
else:
    from bringup.wait_for_topics import main


if __name__ == "__main__":
    sys.exit(main())
