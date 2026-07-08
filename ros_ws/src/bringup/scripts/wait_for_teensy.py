#!/usr/bin/env python3
"""Wrapper for the installable Teensy wait helper."""

import importlib.util
import sys
from pathlib import Path


def _load_source_main():
    module_path = Path(__file__).resolve().parents[1] / "bringup" / "wait_for_teensy.py"
    spec = importlib.util.spec_from_file_location("bringup_wait_for_teensy_source", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.main


try:
    from bringup.wait_for_teensy import main
except ModuleNotFoundError:
    main = _load_source_main()

if __name__ == "__main__":
    sys.exit(main())
