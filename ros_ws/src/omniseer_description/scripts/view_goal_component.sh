 # Visual inspection of select_component_goals
 montage \
  -label 'Goal Component' goals_by_component.ppm \
  -label 'Overlay'  goals_overlay.ppm \
  -tile 2x1 -geometry +6+6 -background black -fill white -pointsize 18 \
  goal_montage.png && xdg-open goal_montage.png || echo "See goal_montage.png"
