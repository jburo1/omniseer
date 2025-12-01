# Visual inspection of /frontier.cpp's compute_frontier_mask output
 montage \
  -label 'Free'     free_mask.pgm \
  -label 'Unknown'  unknown_mask.pgm \
  -label 'Frontier' frontier_mask.pgm \
  -label 'Overlay'  frontier_overlay.ppm \
  -tile 2x2 -geometry +6+6 -background black -fill white -pointsize 18 \
  frontier_montage.png && xdg-open frontier_montage.png || echo "See frontier_montage.png"
