import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
FIRMWARE = ROOT / "firmware/robomaster_c_xelerator_t2"


def test_c_controller_protocol_and_fail_safe_state_machine(tmp_path):
    compiler = next((shutil.which(name) for name in ("cc", "clang", "gcc") if shutil.which(name)), None)
    if compiler is None:
        pytest.skip("native C compiler unavailable")
    output = tmp_path / "chassis_controller_host_test"
    subprocess.run([
        compiler,
        "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-I", str(FIRMWARE / "Core/Inc"),
        str(ROOT / "tests/c/test_chassis_controller.c"),
        str(FIRMWARE / "Core/Src/chassis_kinematics.c"),
        str(FIRMWARE / "Core/Src/chassis_controller.c"),
        str(FIRMWARE / "Core/Src/pi_c_protocol.c"),
        "-lm", "-o", str(output),
    ], check=True)
    subprocess.run([str(output)], check=True)
