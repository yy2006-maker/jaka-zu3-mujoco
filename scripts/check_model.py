#!/usr/bin/env python3
"""Check the Zu 3, gripper actuation, and the deformable rag."""

from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "jaka_zu3_mujoco.xml"
JOINTS = tuple(f"joint_{i}" for i in range(1, 8))
HOME = (0.0, 0.5, 0.8, 0.0, 0.0, 0.0, 0.0)
GRIPPER_CLOSE = 0.0
GRIPPER_OPEN = 0.9


def set_free_body_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
    position: tuple[float, float, float],
) -> None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise RuntimeError(f"Missing body: {body_name}")
    for joint_id in range(model.njnt):
        if (
            model.jnt_bodyid[joint_id] == body_id
            and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        ):
            address = model.jnt_qposadr[joint_id]
            data.qpos[address : address + 3] = position
            data.qpos[address + 3 : address + 7] = (1.0, 0.0, 0.0, 0.0)
            return
    raise RuntimeError(f"Body has no free joint: {body_name}")


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    for index, name in enumerate(JOINTS):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator_{name}"
        )
        if joint_id < 0 or actuator_id < 0:
            raise RuntimeError(f"Missing joint or actuator: {name}")
        data.qpos[model.jnt_qposadr[joint_id]] = HOME[index]
        data.ctrl[actuator_id] = HOME[index]

    rag_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, "rag")
    if model.nflex != 1 or rag_id < 0:
        raise RuntimeError("Expected one deformable flex named 'rag'")
    if model.nu != 7:
        raise RuntimeError("Expected six arm actuators and one gripper actuator")

    # Robot and gripper meshes collide with the environment through mask 2/1;
    # robot-to-robot contacts remain excluded by the XML contact masks/rules.
    for index in range(7):
        visual_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"Link_{index}_visual"
        )
        if visual_id < 0:
            raise RuntimeError(f"Missing visual geometry for Link_{index}")
        if model.geom_contype[visual_id] != 2 or model.geom_conaffinity[visual_id] != 1:
            raise RuntimeError(
                f"Robot collision mask is not 2/1: Link_{index}"
            )

    for name in ("fixed_housing", "fixed_hardware_a", "fixed_hardware_b", "moving"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"gripper_{name}_visual")
        if geom_id < 0:
            raise RuntimeError(f"Missing gripper part: {name}")
        if model.geom_contype[geom_id] != 2 or model.geom_conaffinity[geom_id] != 1:
            raise RuntimeError(f"Gripper collision mask is not 2/1: {name}")
    base = model.body("gripper_base").id
    jaw = model.body("gripper_moving_jaw").id
    if model.body_parentid[base] != model.body("Link_6").id or model.body_parentid[jaw] != base:
        raise RuntimeError("Gripper must be mounted on Link_6 with one moving child jaw")
    joint7 = model.joint("joint_7").id
    if model.jnt_bodyid[joint7] != jaw:
        raise RuntimeError("joint_7 must actuate the moving jaw, not rotate the whole tool")
    if not np.allclose(model.jnt_axis[joint7], (0.0, -1.0, 0.0), atol=1e-6):
        raise RuntimeError("joint_7 direction must open away from the fixed jaw as its value increases")

    for removed_name in ("tool_contact_pad", "tool0"):
        object_type = (
            mujoco.mjtObj.mjOBJ_SITE
            if removed_name == "tool0"
            else mujoco.mjtObj.mjOBJ_GEOM
        )
        if mujoco.mj_name2id(model, object_type, removed_name) >= 0:
            raise RuntimeError(
                f"Obsolete contact object is still present: {removed_name}"
            )

    set_free_body_pose(model, data, "body_obj_mug_5", (0.34, 0.10, 0.85))
    set_free_body_pose(model, data, "body_obj_plate_11", (0.62, -0.12, 0.807))
    mujoco.mj_forward(model, data)

    # Let the cloth settle onto the table. Its table contact is retained even
    # though all robot collision geometry has been removed.
    for _ in range(1000):
        mujoco.mj_step(model, data)
    rag_contacts = sum(1 for contact in data.contact if rag_id in contact.flex)
    rag_start = model.flex_vertadr[rag_id]
    rag_end = rag_start + model.flex_vertnum[rag_id]
    rag_z = data.flexvert_xpos[rag_start:rag_end, 2]
    if rag_contacts == 0 or not (0.79 <= float(rag_z.mean()) <= 0.81):
        raise RuntimeError(
            f"Rag did not settle on the tabletop: mean z={float(rag_z.mean()):.4f}, "
            f"flex contacts={rag_contacts}"
        )

    # Exercise the actual actuator in both directions; do not teleport qpos.
    actuator7 = model.actuator("actuator_joint_7").id
    q7_address = model.jnt_qposadr[joint7]
    results = []
    for target in (GRIPPER_CLOSE, GRIPPER_OPEN):
        start = float(data.ctrl[actuator7])
        steps = round(1.0 / model.opt.timestep)
        for step in range(steps):
            t = (step + 1) / steps
            blend = t * t * (3.0 - 2.0 * t)
            data.ctrl[actuator7] = start + (target - start) * blend
            mujoco.mj_step(model, data)
        for _ in range(steps):
            mujoco.mj_step(model, data)
        actual = float(data.qpos[q7_address])
        if abs(actual - target) > 0.02:
            raise RuntimeError(f"Gripper failed to track target {target}: actual={actual:.5f}")
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise RuntimeError("Non-finite state during gripper movement")
        rotation = data.xmat[base].reshape(3, 3).T @ data.xmat[jaw].reshape(3, 3)
        results.append((actual, rotation.copy()))
    swept_angle = np.arccos(np.clip((np.trace(results[0][1].T @ results[1][1]) - 1) / 2, -1, 1))
    if not (0.85 <= swept_angle <= 0.95):
        raise RuntimeError(f"Moving jaw did not rotate relative to the base: {swept_angle}")
    if np.any(data.warning.number):
        raise RuntimeError(f"MuJoCo warnings during simulation: {data.warning.number}")
    rag_z = data.flexvert_xpos[rag_start:rag_end, 2]
    if not (0.79 <= float(rag_z.mean()) <= 0.81):
        raise RuntimeError("Rag did not remain on the tabletop during gripper movement")

    print(f"MuJoCo {mujoco.__version__}")
    print(f"Model: {MODEL_PATH}")
    print(f"Controlled joints: 6 arm + 1 gripper; actuators: {model.nu}, geoms: {model.ngeom}")
    print(f"Gripper close/open: {results[0][0]:.4f} -> {results[1][0]:.4f} rad; jaw sweep: {swept_angle:.4f} rad")
    print(
        f"Deformable rag: {model.flex_vertnum[rag_id]} vertices, "
        f"{rag_contacts} tabletop contact points"
    )
    # A deterministic collision probe: temporarily place the mug at the
    # gripper base and verify that the enabled masks produce contacts.
    mug_body = model.body("body_obj_mug_5").id
    mug_joint = next(
        joint_id
        for joint_id in range(model.njnt)
        if model.jnt_bodyid[joint_id] == mug_body
        and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
    )
    mug_qpos = model.jnt_qposadr[mug_joint]
    data.qpos[mug_qpos : mug_qpos + 3] = data.xpos[base]
    data.qpos[mug_qpos + 3 : mug_qpos + 7] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    mug_gripper_contacts = 0
    for contact in data.contact:
        body1 = model.geom_bodyid[contact.geom1]
        body2 = model.geom_bodyid[contact.geom2]
        if {body1, body2} & {base, jaw} and model.body("object_mug_5").id in {body1, body2}:
            mug_gripper_contacts += 1
    if mug_gripper_contacts == 0:
        raise RuntimeError("Enabled collision masks produced no mug/gripper contact")

    print(f"Mug/gripper collision probe: {mug_gripper_contacts} contacts")
    print("Robot and gripper collision geometry: enabled; rag model: ready")


if __name__ == "__main__":
    main()
