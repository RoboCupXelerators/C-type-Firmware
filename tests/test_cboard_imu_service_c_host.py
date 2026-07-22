import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/robomaster_c_xelerator_t2"


def test_cboard_imu_service_native(tmp_path):
    compiler = next((shutil.which(name) for name in ("cc", "clang", "gcc") if shutil.which(name)), None)
    if compiler is None:
        pytest.skip("native C compiler unavailable")
    output = tmp_path / "cboard_imu_service_test"
    subprocess.run([
        compiler,
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-pthread", "-DCBOARD_IMU_TEST=1",
        "-I", str(ROOT / "tests/c/cboard_imu_mocks"),
        "-I", str(FIRMWARE / "Core/Inc"),
        str(ROOT / "tests/c/test_cboard_imu_service.c"),
        str(FIRMWARE / "Core/Src/cboard_imu.c"),
        "-lm", "-o", str(output),
    ], check=True)
    subprocess.run([str(output)], check=True)
