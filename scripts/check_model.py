#!/usr/bin/env python3
"""Check that the Zu 3 XML and its downloaded assets are usable."""

from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "jaka_zu3_mujoco.xml"
JOINTS = tuple(f"joint_{i}" for i in range(1, 7))


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    for name in JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator_{name}"
        )
        if joint_id < 0 or actuator_id < 0:
            raise RuntimeError(f"Missing joint or actuator: {name}")

    mujoco.mj_forward(model, data)
    tool_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tool0")
    print(f"MuJoCo {mujoco.__version__}")
    print(f"Model: {MODEL_PATH}")
    print(f"Joints: {len(JOINTS)}, actuators: {model.nu}, geoms: {model.ngeom}")
    print("tool0:", [round(float(value), 4) for value in data.site_xpos[tool_id]])
    print("Model and downloaded assets are ready.")


if __name__ == "__main__":
    main()
