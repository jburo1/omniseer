# Visual inspection of the frontier_e2e pipeline artifacts with dilated goal markers
tmp_work_dir=$(mktemp -d)
trap 'rm -rf "$tmp_work_dir"' EXIT

GOAL_RADIUS=${GOAL_RADIUS:-3}

maybe_dilate() {
  local mode=$1 in_path=$2 out_path=$3
  if command -v python3 >/dev/null 2>&1; then
    python3 - "$GOAL_RADIUS" "$in_path" "$out_path" "$mode" <<'PY'
import sys

radius = max(0, int(sys.argv[1]))
src = sys.argv[2]
dst = sys.argv[3]
mode = sys.argv[4]

def read_ppm(path):
    with open(path, 'rb') as f:
        if f.readline().strip() != b'P6':
            raise SystemExit(f"{path}: unsupported PPM type (need binary P6)")
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                raise SystemExit(f"{path}: truncated header")
            line = line.strip()
            if not line or line.startswith(b'#'):
                continue
            tokens.extend(line.split())
        w, h, maxval = map(int, tokens[:3])
        if maxval != 255:
            raise SystemExit(f"{path}: unsupported maxval {maxval} (expected 255)")
        data = bytearray(f.read())
    expected = w * h * 3
    if len(data) != expected:
        raise SystemExit(f"{path}: pixel payload mismatch (expected {expected}, got {len(data)})")
    return w, h, data

def write_ppm(path, w, h, data):
    with open(path, 'wb') as f:
        f.write(f"P6\n{w} {h}\n255\n".encode())
        f.write(data)

def is_goal_pixel(r, g, b):
    if mode == "overlay":
        return r > g + 10 and r > b + 10
    return (r | g | b) != 0

w, h, pixels = read_ppm(src)
if radius <= 0:
    write_ppm(dst, w, h, pixels)
    sys.exit(0)

out = bytearray(pixels)

for y in range(h):
    base = y * w * 3
    for x in range(w):
        idx = base + x * 3
        r = pixels[idx]
        g = pixels[idx + 1]
        b = pixels[idx + 2]
        if not is_goal_pixel(r, g, b):
            continue
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                nx = x + dx
                ny = y + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                nidx = (ny * w + nx) * 3
                if mode == "overlay":
                    out[nidx] = max(out[nidx], 255)
                    out[nidx + 1] = min(out[nidx + 1], 16)
                    out[nidx + 2] = min(out[nidx + 2], 16)
                else:
                    out[nidx] = r
                    out[nidx + 1] = g
                    out[nidx + 2] = b

write_ppm(dst, w, h, out)
PY
  else
    cp "$in_path" "$out_path"
  fi
}

stage3_overlay="${tmp_work_dir}/stage3_goals_prerank_overlay_dilated.ppm"
stage3_labels="${tmp_work_dir}/stage3_goals_prerank_by_component_dilated.ppm"
stage4_overlay="${tmp_work_dir}/stage4_goals_ranked_overlay_dilated.ppm"
stage4_labels="${tmp_work_dir}/stage4_goals_ranked_by_order_dilated.ppm"

maybe_dilate overlay stage3_goals_prerank_overlay.ppm "$stage3_overlay"
maybe_dilate label   stage3_goals_prerank_by_component.ppm "$stage3_labels"
maybe_dilate overlay stage4_goals_ranked_overlay.ppm "$stage4_overlay"
maybe_dilate label   stage4_goals_ranked_by_order.ppm "$stage4_labels"

if [ "${GOAL_RADIUS:-0}" -gt 0 ]; then
  goal_suffix=" (radius ${GOAL_RADIUS})"
else
  goal_suffix=""
fi

montage \
  -label 'Stage 0: Costmap'                  stage0_costmap.pgm \
  -label 'Stage 1: Frontier Mask'            stage1_frontier_mask.pgm \
  -label 'Stage 1: Frontier Overlay'         stage1_frontier_overlay.ppm \
  -label 'Stage 2: Component Rims'           stage2_components_rims.ppm \
  -label 'Stage 2: Rims Overlay'             stage2_components_overlay.ppm \
  -label "Stage 3: Component Goals" "$stage3_labels" \
  -label "Stage 3: Goals Overlay"      "$stage3_overlay" \
  -label "Stage 4: Ranked Goals"       "$stage4_labels" \
  -label "Stage 4: Ranked Overlay"     "$stage4_overlay" \
  -label 'Stage 5: Top Goal IG Rays'    stage5_top_goal_ig_rays.ppm \
  -tile 4x3 -geometry +6+6 -background black -fill white -pointsize 18 \
  frontier_e2e_montage.png && xdg-open frontier_e2e_montage.png || echo "See frontier_e2e_montage.png"
