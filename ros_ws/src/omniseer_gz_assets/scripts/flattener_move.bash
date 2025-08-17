ROOT="/home/ws/ros_ws/src/omniseer_gz_assets/models"

find "$ROOT" -mindepth 2 -maxdepth 2 -type d -regextype posix-extended -regex '.*/[0-9]+$' \
| while read -r verdir; do
  modeldir="$(dirname "$verdir")"
  if [[ -f "$verdir/model.config" || -f "$verdir/model.sdf" ]]; then
    echo "Flatten: $verdir -> $modeldir"

    rsync -a --ignore-existing --remove-source-files "$verdir"/ "$modeldir"/

    find "$verdir" -type d -empty -delete
    
    rmdir "$verdir" 2>/dev/null || true
  fi
done
