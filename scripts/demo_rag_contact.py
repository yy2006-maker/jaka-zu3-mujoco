#!/usr/bin/env python3
"""Move the Zu 3 tool pad down to the simulated cloth and back up."""

from __future__ import annotations

import time
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "jaka_zu3_mujoco.xml"
JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
HOME = (0.0, 0.5, 0.8, 0.0, 0.0, 0.0)
# Solved for tool0 near the center of the rag, with Link_3..Link_6 proxies
# disabled until they are replaced by table-clear simplified collision shapes.
RAG_TOUCH = (-0.648, -0.042, -0.961, -0.699, 0.931, 1.272)


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

        def run_for(duration: float, targets: tuple[float, ...] | None = None,
                    start: tuple[float, ...] | None = None) -> bool:
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
                viewer.sync()
                time.sleep(dt)
            return True

        print("Letting the rag settle on the table for 2 seconds.")
        if not run_for(2.0):
            return
        print("Moving the orange tool pad toward the rag. Watch for contact markers.")
        if not run_for(5.0, RAG_TOUCH, HOME):
            return
        print("Holding near the rag for 2 seconds.")
        if not run_for(2.0):
            return
        print("Retracting to the raised home pose.")
        run_for(5.0, HOME, RAG_TOUCH)


if __name__ == "__main__":
    main()
