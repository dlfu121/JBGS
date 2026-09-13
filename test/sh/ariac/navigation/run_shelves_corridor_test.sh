#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
# The execution environment may export MUJOCO_GL=egl even without a device
# display.  Use OSMesa for this headless regression; override explicitly with
# SHELVES_MUJOCO_GL when a real display/context is desired.
export MUJOCO_GL="${SHELVES_MUJOCO_GL:-osmesa}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-${ROOT}/test/result/.mplconfig}"
mkdir -p "${MPLCONFIGDIR}"

cd "${ROOT}"
exec python3.8 test/py/ariac/navigation/test_shelves_corridors.py "$@"
