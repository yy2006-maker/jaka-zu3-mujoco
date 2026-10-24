# JAKA Zu 3 in MuJoCo

This repository contains a MuJoCo scene for the six-joint JAKA Zu 3, a tabletop, a deformable cloth patch, a mug, and a plate. It includes the seven-link JAKA geometry (`Link_0.STL` through `Link_6.STL`) and a viewer for testing joint-position control.

## Quick start (Ubuntu)

You need Git, Python 3.10, and `pip`. The required meshes and scene assets are already tracked in this repository, so cloning it does not require a second asset download.

```bash
git clone https://github.com/yy2006-maker/jaka-zu3-mujoco.git
cd jaka-zu3-mujoco

# Optional: create and activate a clean Conda environment.
conda create -n jaka_mujoco python=3.10 -y
conda activate jaka_mujoco

python -m pip install -r requirements.txt
python scripts/check_model.py
python view_jaka_zu3.py
```

In the MuJoCo window, press **Tab** to show or hide the left panel and **Shift+Tab** to show or hide the right panel. Joint target controls are in the right panel under **Control**. Change targets gradually; the robot joints use radians.

## What is included

- `jaka_zu3_mujoco.xml`: the six-joint robot, position actuators, blue deformable cloth, and scene includes.
- `view_jaka_zu3.py`: starts the model in a raised pose, places the mug and plate on the table, and opens the viewer.
- `assets/jaka_zu3_meshes/`: the seven-link JAKA Zu 3 mesh set used for rendering.
- `assets/tabletop/` and `assets/objaverse/`: the table, mug, and plate used by the scene.
- `scripts/fetch_assets.sh`: optional recovery script for recreating the asset directory if a user intentionally removes it; a normal clone does not need to run it.

The rag is a procedural MuJoCo `flexcomp` cloth patch (165 vertices), not a scanned or textured rag model. It falls onto the tabletop and can deform against the tabletop. All JAKA meshes are currently visual-only: no robot collision proxy, tool pad, contact site, or automatic contact demo is included. The mug, plate, table, and rag assets remain in the scene for later work. The original tutorial notebooks demonstrate a different Robotis OMY robot and are not Zu 3 control code.

## Upstream assets

The download script uses these public source repositories:

- JAKA ROS 2 description: <https://github.com/JAKARobotics/jaka_ros2>
- LeRobot MuJoCo tutorial: <https://github.com/jeongeun980906/lerobot-mujoco-tutorial>

The assets in this repository originate from the JAKA ROS 2 description and the LeRobot MuJoCo tutorial. The JAKA source copy used to prepare this project has no clear redistribution license (`package.xml` says `TODO`, and no license file was found), and the tutorial assets have their own upstream terms. Before making this repository public, verify that you have permission to redistribute these files. If you cannot verify that permission, remove the corresponding files and use the optional download script instead.

## Current validation

Tested with Python 3.10 and MuJoCo 3.1.6 on Ubuntu. `scripts/check_model.py` verifies the XML and assets load, confirms the robot meshes are visual-only, and simulates the rag settling onto the tabletop. The project is a work in progress; joint dynamics and collision geometry have not been calibrated against the physical robot.
