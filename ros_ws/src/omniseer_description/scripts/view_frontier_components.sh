# Visual inspection of /frontier.cpp's compute_frontier_mask output
 montage \
  -label 'Rims'     components_rims.ppm \
  -label 'Frontier' frontier_mask.pgm \
  -label 'Rims Overlay'  rims_overlay.ppm \
  -tile 2x2 -geometry +6+6 -background black -fill white -pointsize 18 \
  frontier_montage.png && xdg-open frontier_montage.png || echo "See frontier_montage.png"
