#!/usr/bin/env python3
"""Prepare the replacement gripper assembly supplied by the user.

STL has no units, bodies, joints, or CAD assembly constraints. This
source-specific conversion keeps every disconnected shell from the new export,
converts millimetres to metres, and assigns one long shell to the seventh
MuJoCo joint. The assignment is recorded in assembly_metadata.json.
"""

from __future__ import annotations

import hashlib
import json
import struct
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets" / "gripper"
SOURCE = ASSETS / "source_assembly.stl"
SOURCE_SHA256 = "429be112c2d59e7654cafee41cc1e2e88e466696d6c3bdbbb32cbf0905902385"
STL_DTYPE = np.dtype(
    [
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ]
)

# The circular interface is the 60 mm face of component 25, normal to source X
# (not the side-facing circular motor cover, which is normal to source Z).
# Its robot-facing face is near x=0.58 mm, centred near y=36.67, z=1024 mm.
# Map source +X onto the Link_6 outward +Z direction. Source +Y and +Z map onto
# the flange's local +X and +Y axes, respectively.
MOUNT_ORIGIN_MM = np.array([0.576632, 36.66838, 1024.0])
SOURCE_TO_MOUNT = np.array(
    [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]
)

# The servo shaft passes through the circular rotor centred around source
# (55.5, 36.65) mm; z=1049 mm is its outer face in this STL.
MOVING_PIVOT_MM = np.array([55.5, 36.65, 1049.0])

# The new STL has 27 disconnected shells. Components 0, 1, 12, 13, and 25
# form the housing/plates; 2..11 and 14..24 are visible fasteners; 26 is the
# long member assigned to the illustrative open/close hinge.
FIXED_HOUSING_RANKS = (0, 1, 12, 13, 25)
FIXED_HARDWARE_RANKS = tuple(range(2, 12)) + tuple(range(14, 25))
MOVING_RANK = 26

# The supplied assembly pose is used as qpos=0. Joint axis and travel are
# preview assumptions because STL does not encode the SolidWorks mates.
OPEN_PRE_ROTATION_RAD = 0.0


def component_labels(triangles: np.ndarray) -> np.ndarray:
    """Return a descending-triangle-count component rank for every triangle."""

    vertices, inverse = np.unique(
        np.round(triangles.reshape(-1, 3), 3), axis=0, return_inverse=True
    )
    faces = inverse.reshape(-1, 3)
    parent = np.arange(len(vertices))

    def root(vertex: int) -> int:
        while parent[vertex] != vertex:
            parent[vertex] = parent[parent[vertex]]
            vertex = int(parent[vertex])
        return vertex

    for a, b, c in faces:
        root_a = root(int(a))
        parent[root(int(b))] = root_a
        parent[root(int(c))] = root_a

    roots = np.array([root(int(face[0])) for face in faces])
    _, labels = np.unique(roots, return_inverse=True)
    order = np.argsort(np.bincount(labels))[::-1]
    rank = np.empty(len(order), dtype=int)
    rank[order] = np.arange(len(order))
    return rank[labels]


def rotate_z(points: np.ndarray, angle: float) -> np.ndarray:
    """Rotate an (n, 3) array around its local z axis."""

    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return points @ rotation.T


def write_stl(path: Path, triangles: np.ndarray, header: str) -> None:
    """Write triangles as a binary STL with valid computed normals."""

    triangles = np.asarray(triangles, dtype=np.float64)
    records = np.zeros(len(triangles), dtype=STL_DTYPE)
    records["vertices"] = triangles.astype(np.float32)
    normals = np.cross(
        triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    )
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    records["normal"] = (normals / np.maximum(lengths, 1e-20)).astype(np.float32)
    path.write_bytes(
        header.encode("ascii", "replace")[:80].ljust(80, b" ")
        + struct.pack("<I", len(records))
        + records.tobytes()
    )


def local_mesh(triangles: np.ndarray, moving: bool) -> np.ndarray:
    """Convert source millimetres to metres in the selected body frame."""

    origin = MOVING_PIVOT_MM if moving else MOUNT_ORIGIN_MM
    local = (triangles.astype(np.float64) - origin) @ SOURCE_TO_MOUNT.T * 0.001
    if moving:
        local = rotate_z(local.reshape(-1, 3), OPEN_PRE_ROTATION_RAD).reshape(
            local.shape
        )
    return local


def bounds(triangles: np.ndarray) -> list[list[float]]:
    vertices = triangles.reshape(-1, 3)
    return [vertices.min(0).tolist(), vertices.max(0).tolist()]


def main() -> None:
    raw = SOURCE.read_bytes()
    actual_hash = hashlib.sha256(raw).hexdigest()
    if actual_hash != SOURCE_SHA256:
        raise RuntimeError(
            "source_assembly.stl is not the supplied 装配体2.STL; "
            f"expected {SOURCE_SHA256}, got {actual_hash}"
        )

    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * count:
        raise RuntimeError("Expected a binary STL with an exact triangle count")
    triangles = np.frombuffer(raw, dtype=STL_DTYPE, count=count, offset=84)[
        "vertices"
    ].copy()
    labels = component_labels(triangles)
    component_count = int(labels.max()) + 1
    if component_count != 27:
        raise RuntimeError(f"Expected 27 disconnected shells, found {component_count}")

    def select(ranks: tuple[int, ...]) -> np.ndarray:
        selected = triangles[np.isin(labels, ranks)]
        if len(selected) == 0:
            raise RuntimeError(f"No triangles selected for component ranks {ranks}")
        return selected

    fixed_housing = local_mesh(select(FIXED_HOUSING_RANKS), moving=False)
    fixed_hardware_a = local_mesh(select(tuple(range(2, 12))), moving=False)
    fixed_hardware_b = local_mesh(select(tuple(range(14, 25))), moving=False)
    moving_jaw = local_mesh(triangles[labels == MOVING_RANK], moving=True)

    # Remove files from the earlier provisional assembly so a clone cannot
    # accidentally keep loading a stale mesh. The source STL is retained.
    for old_name in ("fixed_jaw.stl", "servo_body.stl", "mount_screw.stl"):
        old_path = ASSETS / old_name
        if old_path.exists():
            old_path.unlink()

    outputs = {
        "fixed_housing": (fixed_housing, FIXED_HOUSING_RANKS),
        "fixed_hardware_a": (fixed_hardware_a, tuple(range(2, 12))),
        "fixed_hardware_b": (fixed_hardware_b, tuple(range(14, 25))),
        "moving_jaw": (moving_jaw, (MOVING_RANK,)),
    }
    for name, (mesh, ranks) in outputs.items():
        write_stl(
            ASSETS / f"{name}.stl",
            mesh,
            f"JAKA Zu 3 gripper {name}; metres; source component ranks {ranks}",
        )
        print(f"{name}: {len(mesh)} triangles; bounds={bounds(mesh)}")

    metadata = {
        "source_file": "source_assembly.stl",
        "original_filename": "装配体2.STL",
        "sha256": SOURCE_SHA256,
        "source_triangles": int(count),
        "source_components": component_count,
        "units_assumption": "Source millimetres; generated mesh vertices in metres",
        "mount_origin_mm": MOUNT_ORIGIN_MM.tolist(),
        "source_to_mount_rotation": SOURCE_TO_MOUNT.tolist(),
        "moving_pivot_mm": MOVING_PIVOT_MM.tolist(),
        "moving_pivot_in_mount_m": ((MOVING_PIVOT_MM - MOUNT_ORIGIN_MM) @ SOURCE_TO_MOUNT.T * 0.001).tolist(),
        "moving_pre_rotation_rad": OPEN_PRE_ROTATION_RAD,
        "parts": {
            "fixed_housing": {
                "component_ranks": list(FIXED_HOUSING_RANKS),
                "triangles": int(len(fixed_housing)),
                "frame": "Link_6 flange mount frame",
                "bounds_m": bounds(fixed_housing),
            },
            "fixed_hardware_a": {
                "component_ranks": list(range(2, 12)),
                "triangles": int(len(fixed_hardware_a)),
                "frame": "Link_6 flange mount frame",
                "bounds_m": bounds(fixed_hardware_a),
            },
            "fixed_hardware_b": {
                "component_ranks": list(range(14, 25)),
                "triangles": int(len(fixed_hardware_b)),
                "frame": "Link_6 flange mount frame",
                "bounds_m": bounds(fixed_hardware_b),
            },
            "moving_jaw": {
                "component_ranks": [MOVING_RANK],
                "triangles": int(len(moving_jaw)),
                "frame": "joint_7 pivot frame",
                "bounds_m": bounds(moving_jaw),
            },
        },
        "scope": "Complete user-provided gripper assembly; all 27 shells retained",
        "simulation_assumptions": {
            "mount": "60 mm circular face on component 25, source normal +X mapped to Link_6 local +Z; bolt fit unverified",
            "joint_7_axis": "Source Z motor shaft mapped to Link_6 local -Y so positive joint_7 opens away from the fixed jaw",
            "joint_7_range_rad": [0.0, 0.9],
            "joint_7_range_note": "0 is the closed reference pose; increasing opens the jaw; visual preview range, not measured servo travel",
            "fixed_mass_kg": 0.45,
            "moving_jaw_mass_kg": 0.08,
            "inertia": "Uniform AABB estimates, not manufacturer mass/inertia",
            "collision": "Robot and gripper mesh convex hulls collide with environment using contype 2 and conaffinity 1; internal self-contact pairs excluded; physical geometry is an approximation",
        },
    }
    (ASSETS / "assembly_metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print("source triangles:", count)
    print("component ranks: fixed housing", FIXED_HOUSING_RANKS)
    print("component ranks: fixed hardware", FIXED_HARDWARE_RANKS)
    print("component rank: moving jaw", MOVING_RANK)
    print("joint_7 reference: 0 rad closed; positive values open the jaw")


if __name__ == "__main__":
    main()
