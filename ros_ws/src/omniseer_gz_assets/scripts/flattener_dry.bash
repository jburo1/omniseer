ROOT="/home/ws/ros_ws/src/omniseer_gz_assets/models"

# DRY RUN: see what would be moved
find "$ROOT" -mindepth 2 -maxdepth 2 -type d -regextype posix-extended -regex '.*/[0-9]+$' \
| while read -r verdir; do
  modeldir="$(dirname "$verdir")"
  if [[ -f "$verdir/model.config" || -f "$verdir/model.sdf" ]]; then
    echo "Would move contents of: $verdir -> $modeldir"
    rsync -a -n --info=NAME "$verdir"/ "$modeldir"/
  fi
done
