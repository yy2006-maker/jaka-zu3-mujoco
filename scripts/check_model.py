#!/usr/bin/env python3
"""Check that the Zu 3 XML and its downloaded assets are usable."""

from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "jaka_zu3_mujoco.xml"
JOINTS = tuple(f"joint_{i}" for i in range(1, 7))
HOME = (0.0, 0.5, 0.8, 0.0, 0.0, 0.0)


def set_free_body_pose(model: mujoco.MjModel, data: mujoco.MjData,
                       body_name: str, position: tuple[float, float, float]) -> None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise RuntimeError(f"Missing body: {body_name}")
    for joint_id in range(model.njnt):
        if (model.jnt_bodyid[joint_id] == body_id
                and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE):
            address = model.jnt_qposadr[joint_id]
            data.qpos[address:address + 3] = position
            data.qpos[address + 3:address + 7] = (1.0, 0.0, 0.0, 0.0)
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
    enabled_collision_geoms = (
        "Link_0_collision", "Link_1_collision", "Link_2_collision",
        "tool_contact_pad",
    )
    for name in enabled_collision_geoms:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise RuntimeError(f"Missing collision geometry: {name}")
        if model.geom_contype[geom_id] == 0 or model.geom_conaffinity[geom_id] == 0:
            raise RuntimeError(f"Collision is disabled for {name}")
    for index in range(3, 7):
        geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"Link_{index}_collision"
        )
        if geom_id < 0 or model.geom_contype[geom_id] != 0:
            raise RuntimeError(f"Expected Link_{index} proxy to stay disabled")

    set_free_body_pose(model, data, "body_obj_mug_5", (0.34, 0.10, 0.85))
    set_free_body_pose(model, data, "body_obj_plate_11", (0.62, -0.12, 0.807))

    mujoco.mj_forward(model, data)
    tool_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tool0")
    # Let the cloth settle onto the table and verify a flex/table contact.
    for _ in range(500):
        mujoco.mj_step(model, data)
    rag_contacts = sum(
        1 for contact in data.contact
        if rag_id in contact.flex
    )
    rag_z = data.flexvert_xpos[model.flex_vertadr[rag_id]:
                               model.flex_vertadr[rag_id] + model.flex_vertnum[rag_id], 2]
    if rag_contacts == 0 or not (0.79 <= float(rag_z.mean()) <= 0.81):
        raise RuntimeError(
            f"Rag did not settle on the tabletop: mean z={float(rag_z.mean()):.4f}, "
            f"flex contacts={rag_contacts}"
        )

    # Confirm the orange tool proxy can physically touch the settled cloth by
    # using the same position actuators as the viewer demo.  This is a dynamic
    # check: it catches target poses that only work when qpos is teleported.
    tool_touch_pose = (-0.648, -0.042, -0.961, -0.699, 0.931, 1.272)
    for index, name in enumerate(JOINTS):
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator_{name}"
        )
        data.ctrl[actuator_id] = tool_touch_pose[index]
    for _ in range(round(5.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tool_contact_pad")
    tool_rag_contacts = sum(
        1 for contact in data.contact
        if rag_id in contact.flex and pad_id in contact.geom
    )
    if tool_rag_contacts == 0:
        raise RuntimeError("The tool contact pad did not contact the rag at test pose")

    print(f"MuJoCo {mujoco.__version__}")
    print(f"Model: {MODEL_PATH}")
    print(f"Joints: {len(JOINTS)}, actuators: {model.nu}, geoms: {model.ngeom}")
    print(f"Deformable rag: {model.flex_vertnum[rag_id]} vertices, "
          f"{rag_contacts} tabletop contact points")
    print("tool0:", [round(float(value), 4) for value in data.site_xpos[tool_id]])
    print(f"Tool-pad/rag contacts at test pose: {tool_rag_contacts}")
    print("Robot collision geometry, tool contact pad, and rag are ready.")


if __name__ == "__main__":
    main()
