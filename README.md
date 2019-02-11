This repo contains packages for active path planning for 3D reconstruction. The planning is based on voxblox map representations. Perception simulation is performed in unreal engine 4 worlds.

Some ideas and related literature: [MT Notes](https://docs.google.com/document/d/1gfO_fo0DgKRcGM9M1EXcKeVk7lgDWiqjFwAItRCTF78/edit?usp=sharing)

Available ressources: [data repository](https://www.polybox.ethz.ch/index.php/s/6vhPDINcISbEogg)

# Installation
TODO

# Packages
More detailed information for every package is found in their respective readmes.

* **unreal_cv_ros**
  Simulate 3D perception for MAVs in an unreal engine 4 game.

* **mav_active_3d_planning**
  Sampling based receding horizon active pathplanning framework.
  
# Known issues / todos
- [x] Unreal turn rate is sometimes slow, leading to wrong angles of taken images for high yaw rates (especially standard mode)
- [ ] Next issues
