#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "$ROOT"

export MUJOCO_GL=glfw

python3.8 model/scenes/build_ariac_compat_meshes.py --check
python3.8 model/robot/gen_ariac_robot.py
python3.8 test/py/ariac/model/test_ariac_scene.py
python3.8 test/py/ariac/model/test_robot_sidedness.py
python3.8 test/py/ariac/model/test_inspection_route.py
