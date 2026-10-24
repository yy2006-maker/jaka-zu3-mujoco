#!/usr/bin/env python3
"""Show a visible, measurable contact between the Zu 3 and a deformable rag.

The orange sphere is the collision proxy for the end tool.  During the demo it
turns red when it is in contact with the flex cloth, while the terminal prints
the number of pad/rag contacts and the normal contact force.
"""

from __future__ import annotations

import time
import os
from pathlib import Path

import numpy as np
import mujoco

# Use XWayland on systems where GLFW cannot create a Wayland surface.  This is
# also used by view_jaka_zu3.py and prevents a blank/failed demo window.
os.environ.setdefault("PYGLFW_LIBRARY_VARIANT", "x11")

import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "jaka_zu3_mujoco.xml"
JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
HOME = (0.0, 0.5, 0.8, 0.0, 0.0, 0.0)
# Solved for tool0 near the center of the rag, with Link_3..Link_6 proxies
# disabled until they are replaced by table-clear simplified collision shapes.
RAG_TOUCH = (-0.648, -0.042, -0.961, -0.699, 0.931, 1.272)


def contact_metrics(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    rag_id: int,
    pad_id: int,
) -> tuple[int, float]:
    """Return (number of pad/rag contacts, total 3-D contact force in N)."""
    count = 0
    force_total = 0.0
    wrench = np.zeros(6)
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        if rag_id not in contact.flex or pad_id not in contact.geom:
            continue
        count += 1
        mujoco.mj_contactForce(model, data, contact_index, wrench)
        force_total += sum(value * value for value in wrench[:3]) ** 0.5
    return count, force_total


def set_free_body_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
    position: tuple[float, float, float],
) -> None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    for joint_id in range(model.njnt):
        if (
            model.jnt_bodyid[joint_id] == body_id
            and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        ):
            address = model.jnt_qposadr[joint_id]
            data.qpos[address:address + 3] = position
            data.qpos[address + 3:address + 7] = (1.0, 0.0, 0.0, 0.0)
            return
    raise RuntimeError(f"Could not find a free joint for {body_name}")


def set_joint_targets(
    model: mujoco.MjModel, data: mujoco.MjData, values: tuple[float, ...]
) -> None:
    for name, value in zip(JOINTS, values):
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator_{name}"
        )
        data.ctrl[actuator_id] = value


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    for name, value in zip(JOINTS, HOME):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint_id]] = value
    set_joint_targets(model, data, HOME)
    set_free_body_pose(model, data, "body_obj_mug_5", (0.34, 0.10, 0.85))
    set_free_body_pose(model, data, "body_obj_plate_11", (0.62, -0.12, 0.807))
    # The qpos values above are direct state edits.  Recompute kinematics and
    # contacts before opening the viewer so the first rendered frame is valid.
    mujoco.mj_forward(model, data)

    rag_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, "rag")
    pad_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "tool_contact_pad"
    )
    tool_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tool0")
    if rag_id < 0 or pad_id < 0 or tool_id < 0:
        raise RuntimeError("The rag, tool_contact_pad, or tool0 is missing")

    with mujoco.viewer.launch_passive(
        model, data, show_left_ui=True, show_right_ui=True
    ) as viewer:
        viewer.cam.lookat[:] = [0.42, -0.02, 0.91]
        viewer.cam.distance = 1.8
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        viewer.sync()

        dt = model.opt.timestep
        last_report = -1.0

        def run_for(duration: float, targets: tuple[float, ...] | None = None,
                    start: tuple[float, ...] | None = None) -> bool:
            nonlocal last_report
            steps = round(duration / dt)
            for step in range(steps):
                if not viewer.is_running():
                    return False
                if targets is not None and start is not None:
                    progress = (step + 1) / steps
                    blend = progress * progress * (3.0 - 2.0 * progress)
                    values = tuple(
                        origin + (target - origin) * blend
                        for origin, target in zip(start, targets)
                    )
                    set_joint_targets(model, data, values)
                mujoco.mj_step(model, data)

                pad_contacts, pad_force = contact_metrics(
                    model, data, rag_id, pad_id
                )
                # Make contact unmistakable in the viewer: orange is free,
                # red means the tool proxy is physically touching the rag.
                model.geom_rgba[pad_id] = (
                    1.0, 0.05, 0.02, 1.0
                ) if pad_contacts else (1.0, 0.25, 0.08, 1.0)
                elapsed = step * dt
                if elapsed - last_report >= 0.25 or step == steps - 1:
                    tool_position = data.site_xpos[tool_id]
                    rag_start = model.flex_vertadr[rag_id]
                    rag_end = rag_start + model.flex_vertnum[rag_id]
                    rag_height = data.flexvert_xpos[rag_start:rag_end, 2].mean()
                    print(
                        f"  tool0=({tool_position[0]:+.3f}, "
                        f"{tool_position[1]:+.3f}, {tool_position[2]:+.3f}) "
                        f"rag_z={rag_height:.3f} "
                        f"pad-rag contacts={pad_contacts} "
                        f"force={pad_force:.3f} N",
                        flush=True,
                    )
                    last_report = elapsed
                viewer.sync()
                time.sleep(dt)
            return True

        print("Letting the rag settle on the table for 2 seconds.")
        if not run_for(2.0):
            return
        print("Moving the orange tool pad toward the rag. Watch for contact markers.")
        if not run_for(5.0, RAG_TOUCH, HOME):
            return
        print("Holding in contact with the rag for 2 seconds.")
        if not run_for(2.0):
            return
        print("Retracting to the raised home pose.")
        run_for(5.0, HOME, RAG_TOUCH)


if __name__ == "__main__":
    main()
