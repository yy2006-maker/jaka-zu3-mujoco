# JAKA Zu 3 in MuJoCo

This repository contains a self-contained MuJoCo scene for the six-joint JAKA Zu 3 and a viewer for testing joint-position control. The repository includes the seven-link JAKA visual geometry (`Link_0.STL` through `Link_6.STL`) together with the tabletop, mug, and plate used in the current scene.

## Quick start (Ubuntu)

You need Git, Python 3.10, and `pip`. The required meshes and scene assets are already tracked in this repository, so cloning it does not require a second asset download.

```bash
git clone https://github.com/YOUR-USER/YOUR-REPOSITORY.git
cd YOUR-REPOSITORY

# Optional: create and activate a clean Conda environment.
conda create -n jaka_mujoco python=3.10 -y
conda activate jaka_mujoco

python -m pip install -r requirements.txt
python scripts/check_model.py
python view_jaka_zu3.py
```

In the MuJoCo window, press **Tab** to show or hide the left panel and **Shift+Tab** to show or hide the right panel. Joint controls are in the right panel under **Control**.

## What is included

- `jaka_zu3_mujoco.xml`: the six-joint robot definition, inertial properties, position actuators, and scene includes.
- `view_jaka_zu3.py`: starts the model in a raised pose and opens the MuJoCo viewer with both side panels enabled.
- `assets/jaka_zu3_meshes/`: the complete seven-link JAKA Zu 3 visual mesh set.
- `assets/tabletop/` and `assets/objaverse/`: the table, mug, and plate used by the current scene.
- `scripts/fetch_assets.sh`: optional recovery script for recreating the asset directory if a user intentionally removes it; a normal clone does not need to run it.

After cloning, `assets/jaka_zu3_meshes/` contains the full Zu 3 visual link geometry needed by the XML. `scripts/check_model.py` verifies that all six joints, six actuators, and the referenced assets load successfully.

The current robot links are visual-only meshes (`contype="0"`, `conaffinity="0"`). There is no gripper yet, and the arm does not collide with the table or objects. The current stage is suitable for viewing and testing joint motion, not for a pick-and-place task or policy training. The original tutorial notebooks demonstrate a different Robotis OMY robot and are not Zu 3 control code.

## Upstream assets

The download script uses these public source repositories:

- JAKA ROS 2 description: <https://github.com/JAKARobotics/jaka_ros2>
- LeRobot MuJoCo tutorial: <https://github.com/jeongeun980906/lerobot-mujoco-tutorial>

The assets in this repository originate from the JAKA ROS 2 description and the LeRobot MuJoCo tutorial. The JAKA source copy used to prepare this project has no clear redistribution license (`package.xml` says `TODO`, and no license file was found), and the tutorial assets have their own upstream terms. Before making this repository public, verify that you have permission to redistribute these files. If you cannot verify that permission, remove the corresponding files and use the optional download script instead.

## Current validation

Tested with Python 3.10 and MuJoCo 3.1.6 on Ubuntu. The project is a work in progress; joint dynamics and contact geometry have not been calibrated against the physical robot.
