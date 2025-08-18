urls=(
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/JanSport%20Backpack%20Red"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Fire%20Extinguisher"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction%20Barrel"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Cardboard%20box"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/RoboCup%203D%20Simulation%20Ball"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/FIRST%202015%20trash%20can"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Checkerboard%20Plane"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hammer"
  "https://fuel.gazebosim.org/1.0/GoogleResearch/models/Threshold_Porcelain_Teapot_White"
  "https://fuel.gazebosim.org/1.0/GoogleResearch/models/Retail_Leadership_Summit_eCT3zqHYIkX"
  "https://fuel.gazebosim.org/1.0/GoogleResearch/models/Travel_Mate_P_series_Notebook"
  "https://fuel.gazebosim.org/1.0/GoogleResearch/models/Timberland_Mens_Earthkeepers_Stormbuck_Plain_Toe_Oxford"
  "https://fuel.gazebosim.org/1.0/GoogleResearch/models/Kanex_MultiSync_Wireless_Keyboard"
)

for u in "${urls[@]}"; do
  echo "Downloading: $u"
  gz fuel download -u "$u"
done