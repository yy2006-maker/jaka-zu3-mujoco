#!/usr/bin/env python3
"""Check that the Zu 3 scene and the deformable rag load correctly."""

from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "jaka_zu3_mujoco.xml"
JOINTS = tuple(f"joint_{i}" for i in range(1, 7))
HOME = (0.0, 0.5, 0.8, 0.0, 0.0, 0.0)


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

    # Robot meshes are visual-only for now. This keeps the scene stable while
    # retaining the deformable cloth and its tabletop collision.
    for index in range(7):
        visual_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"Link_{index}_visual"
        )
        if visual_id < 0:
            raise RuntimeError(f"Missing visual geometry for Link_{index}")
        if model.geom_contype[visual_id] != 0 or model.geom_conaffinity[visual_id] != 0:
            raise RuntimeError(
                f"Robot visual geometry unexpectedly collides: Link_{index}"
            )

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

    print(f"MuJoCo {mujoco.__version__}")
    print(f"Model: {MODEL_PATH}")
    print(f"Joints: {len(JOINTS)}, actuators: {model.nu}, geoms: {model.ngeom}")
    print(
        f"Deformable rag: {model.flex_vertnum[rag_id]} vertices, "
        f"{rag_contacts} tabletop contact points"
    )
    print("Robot collision geometry: disabled; rag model: ready")


if __name__ == "__main__":
    main()
