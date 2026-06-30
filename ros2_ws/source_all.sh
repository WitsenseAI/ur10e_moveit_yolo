#!/usr/bin/env bash
# Usage: source ros2_ws/source_all.sh   (NOT ./source_all.sh)
#
# Sources the full runtime env: base ROS (Jazzy) -> MoveIt overlay -> THIS workspace.
# This file is SOURCED, so it must not leave `errexit`/`nounset` on in your shell
# (that would close the terminal on the next failing command) — hence no `set -e`.

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# MOVEIT_WS="${MOVEIT_WS:-$HOME/ws_moveit}"

# Overlay order matters: base first, then MoveIt, then THIS workspace LAST so our
# rebuilt overlays (e.g. topic_based_ros2_control) shadow the apt/base versions.
source /opt/ros/jazzy/setup.bash
# if [ -f "$MOVEIT_WS/install/setup.bash" ]; then
#   source "$MOVEIT_WS/install/setup.bash"
# else
#   echo "source_all.sh: WARNING: no MoveIt install at $MOVEIT_WS (set MOVEIT_WS=...)" >&2
# fi
source "$WS_DIR/install/setup.bash"

# Optional extra overlay (machine-specific). Only sourced if present, so this
# script stays portable for anyone cloning the repo. Override with EXTRA_WS=...
EXTRA_WS="${EXTRA_WS:-$HOME/witsense/ugv_core/ros2_ws/install/setup.bash}"
[ -f "$EXTRA_WS" ] && source "$EXTRA_WS"

# Heads-up: ROS python nodes need /usr/bin/python3 (it has rclpy/dcb_core). conda
# or a venv on PATH shadows it and breaks nodes at runtime. Warn, don't auto-fix.
if [ -n "${CONDA_DEFAULT_ENV:-}" ] || [ -n "${VIRTUAL_ENV:-}" ]; then
  echo "source_all.sh: WARNING: conda/venv active ('${CONDA_DEFAULT_ENV:-}${VIRTUAL_ENV:+ venv}')" >&2
  echo "  python3 -> $(command -v python3); run 'conda deactivate' / 'deactivate' for ROS nodes." >&2
fi
