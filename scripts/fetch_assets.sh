#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ASSET_DIR="$ROOT_DIR/assets"
TEMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TEMP_DIR"' EXIT

JAKA_REPO="${JAKA_REPO:-https://github.com/JAKARobotics/jaka_ros2.git}"
LEROBOT_REPO="${LEROBOT_REPO:-https://github.com/jeongeun980906/lerobot-mujoco-tutorial.git}"

if [[ -e "$ASSET_DIR" ]]; then
  required_files=(
    jaka_zu3_meshes/Link_0.STL
    jaka_zu3_meshes/Link_1.STL
    jaka_zu3_meshes/Link_2.STL
    jaka_zu3_meshes/Link_3.STL
    jaka_zu3_meshes/Link_4.STL
    jaka_zu3_meshes/Link_5.STL
    jaka_zu3_meshes/Link_6.STL
    tabletop/object/floor_isaac_style.xml
    tabletop/object/object_table.xml
    tabletop/mesh/light_wood_v3.png
    objaverse/mug_5/model_new.xml
    objaverse/plate_11/model_new.xml
  )
  complete=true
  for file in "${required_files[@]}"; do
    if [[ ! -f "$ASSET_DIR/$file" ]]; then
      complete=false
      break
    fi
  done
  if [[ "$complete" == true ]]; then
    echo "All required assets are already present at $ASSET_DIR."
    exit 0
  fi
  echo "Assets already exist at $ASSET_DIR; move or remove that folder before replacing them." >&2
  exit 1
fi

clone_sparse() {
  local url="$1"
  local dest="$2"
  shift 2
  git clone --depth 1 --filter=blob:none --sparse "$url" "$dest"
  git -C "$dest" sparse-checkout set "$@"
}

echo "Downloading only the JAKA Zu 3 mesh files..."
clone_sparse "$JAKA_REPO" "$TEMP_DIR/jaka_ros2" \
  src/jaka_description/meshes/jaka_zu3_meshes

echo "Downloading the MuJoCo tutorial's tabletop and task objects..."
clone_sparse "$LEROBOT_REPO" "$TEMP_DIR/lerobot-mujoco-tutorial" \
  asset/tabletop/object/floor_isaac_style.xml \
  asset/tabletop/object/object_table.xml \
  asset/tabletop/mesh/light_wood_v3.png \
  asset/objaverse/mug_5 \
  asset/objaverse/plate_11.zip

OBJAVERSE_SRC="$TEMP_DIR/lerobot-mujoco-tutorial/asset/objaverse"
if [[ ! -f "$OBJAVERSE_SRC/plate_11.zip" ]]; then
  echo "Expected plate_11.zip was not found in the tutorial repository." >&2
  exit 1
fi
if ! command -v unzip >/dev/null 2>&1; then
  echo "Please install unzip, then run this script again." >&2
  exit 1
fi

# Build a complete asset tree in a temporary location first so an interrupted
# download cannot leave half-copied assets in the working tree.
STAGE_DIR="$TEMP_DIR/assets"
mkdir -p "$STAGE_DIR/jaka_zu3_meshes"
mkdir -p "$STAGE_DIR/tabletop/object" "$STAGE_DIR/tabletop/mesh"
mkdir -p "$STAGE_DIR/objaverse"

cp -a "$TEMP_DIR/jaka_ros2/src/jaka_description/meshes/jaka_zu3_meshes/." \
  "$STAGE_DIR/jaka_zu3_meshes/"
cp "$TEMP_DIR/lerobot-mujoco-tutorial/asset/tabletop/object/floor_isaac_style.xml" \
  "$STAGE_DIR/tabletop/object/"
cp "$TEMP_DIR/lerobot-mujoco-tutorial/asset/tabletop/object/object_table.xml" \
  "$STAGE_DIR/tabletop/object/"
cp "$TEMP_DIR/lerobot-mujoco-tutorial/asset/tabletop/mesh/light_wood_v3.png" \
  "$STAGE_DIR/tabletop/mesh/"
cp -a "$OBJAVERSE_SRC/mug_5" "$STAGE_DIR/objaverse/"
unzip -q "$OBJAVERSE_SRC/plate_11.zip" -d "$STAGE_DIR/objaverse"

mv "$STAGE_DIR" "$ASSET_DIR"

echo "Assets are ready. Run: python view_jaka_zu3.py"
