import sys, re, json
'''
script to convert nav2 get_costmap service reply to pgm and JSON
'''
def grab(text, pat, cast=float):
    m = re.search(pat, text, re.DOTALL)
    if not m:
        raise SystemExit(f"pattern not found: {pat}")
    return cast(m.group(1))

def main(path):
    txt = open(path, 'r', encoding='utf-8').read()

    # Get metadata
    W   = grab(txt, r"size_x=(\d+)", int)
    H   = grab(txt, r"size_y=(\d+)", int)
    RES = grab(txt, r"resolution=([0-9eE\.\-]+)", float)
    mxy = re.search(r"Point\(x=([-\d.eE]+),\s*y=([-\d.eE]+),\s*z=([-\d.eE]+)\)", txt, re.DOTALL)
    if not mxy:
        raise SystemExit("origin Point(x, y, z) not found")
    OX = float(mxy.group(1))
    OY = float(mxy.group(2))

    m = re.search(r"data=\[([0-9,\s]+)\]", txt, re.DOTALL)
    if not m:
        raise SystemExit("data[...] not found.")
    data = [int(x) for x in m.group(1).replace("\n"," ").split(",") if x.strip()]

    if len(data) != W * H:
        raise SystemExit(f"len(data)={len(data)} but expected {W*H}. ")

    if not all(0 <= v <= 255 for v in data):
        bad = [v for v in data if not (0 <= v <= 255)][:5]
        raise SystemExit(f"values out of range [0,255], examples: {bad}")

    # Write PGM (P5)
    with open("examples/map.pgm", "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode("ascii"))
        f.write(bytes(data))

    # Write metadata JSON
    meta = {"width": W, "height": H, "resolution": RES, "origin": [OX, OY], "frame": "map"}
    with open("examples/map.meta.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    print(f"ok: wrote examples/map.pgm and examples/map.meta.json ({W}x{H})")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: python scripts/to_pgm.py <path/to/costmap.txt>")
    main(sys.argv[1])
