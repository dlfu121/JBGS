#!/usr/bin/env python3
"""VLA-task entry point for randomized ARIAC screwdriver placement.

The implementation lives beside ariac_lab.xml so that its default input and
all scene-relative paths remain unambiguous.  This small entry point keeps
the task-specific command in _ultimate_task/vla, as used by VLA experiments.
"""

import sys
from pathlib import Path


SCENE_SCRIPTS = Path(__file__).resolve().parents[2] / "model" / "scenes"
sys.path.insert(0, str(SCENE_SCRIPTS))

from randomize_ariac_grasp import main  # noqa: E402


if __name__ == "__main__":
    main()
