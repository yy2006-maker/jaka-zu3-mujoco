#!/usr/bin/env python3
"""Open the JAKA Zu 3 MuJoCo model with a centered initial camera."""

import time
import os
from pathlib import Path

# MuJoCo's GLFW viewer can render into only part of the window on some
# Wayland/high-DPI configurations. Use XWayland to get matching logical and
# framebuffer sizes.
os.environ.setdefault("PYGLFW_LIBRARY_VARIANT", "x11")

import mujoco
import mujoco.viewer


MODEL_PATH = Path(__file__).with_name("jaka_zu3_mujoco.xml")


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    # Hold a raised, tucked pose. Zeroing every joint makes this arm lie across
    # the table, which is why the wrist looked as if it had sunk into the floor.
    home = (0.0, 0.5, 0.8, 0.0, 0.0, 0.0)
    for joint_index, joint_name in enumerate(
        ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
    ):
        joint = model.joint(joint_name)
        data.qpos[joint.qposadr] = home[joint_index]
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator_{joint_name}"
        )
        data.ctrl[actuator_id] = home[joint_index]

    # Put the tutorial's mug and plate on top of the table. Their unmodified
    # default poses would both start at the world origin.
    for body_name, position in (
        ("body_obj_mug_5", (0.34, 0.10, 0.85)),
        ("body_obj_plate_11", (0.62, -0.12, 0.807)),
    ):
        body = model.body(body_name)
        free_joint_id = next(
            joint_id
            for joint_id in range(model.njnt)
            if model.jnt_bodyid[joint_id] == body.id
            and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        )
        qpos_address = model.jnt_qposadr[free_joint_id]
        data.qpos[qpos_address : qpos_address + 3] = position
        data.qpos[qpos_address + 3 : qpos_address + 7] = (1, 0, 0, 0)

    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(
        model, data, show_left_ui=True, show_right_ui=True
    ) as viewer:
        # Show the robot and the task objects together.
        viewer.cam.lookat[:] = [0.48, 0.0, 0.93]
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        # Make physical contacts visible while testing the tool against the rag.
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        viewer.sync()

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
