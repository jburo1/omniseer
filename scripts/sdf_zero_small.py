#!/usr/bin/env python3
"""
zero_small.py   – overwrite an SDF/URDF in-place, turning any numeric token
                  whose absolute value is below EPS into 0
usage:  ./zero_small.py robot.sdf [EPS]
"""

import sys, xml.etree.ElementTree as ET, math

EPS = float(sys.argv[2]) if len(sys.argv) > 2 else 1e-6
fname = sys.argv[1]

tree = ET.parse(fname)
root = tree.getroot()

def is_number(tok: str) -> bool:
    try:
        float(tok)
        return True
    except ValueError:
        return False

for el in root.iter():
    # skip <uri> and friends – they often contain version strings or filenames
    if el.tag in {"uri"}:
        continue
    if el.text:
        tokens = el.text.split()
        changed = False
        for i, t in enumerate(tokens):
            if is_number(t) and abs(float(t)) < EPS:
                tokens[i] = "0"
                changed = True
        if changed:
            el.text = " ".join(tokens)

tree.write(fname, encoding="utf-8", xml_declaration=True)
print(f"✓ cleaned {fname} (ε = {EPS})")
