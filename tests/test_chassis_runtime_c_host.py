import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/robomaster_c_xelerator_t2"


def test_chassis_runtime_native_fault_injection(tmp_path):
    output = tmp_path / "chassis_runtime_test"
    command = [
        "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-I", str(FIRMWARE / "Core/Inc"),
        str(ROOT / "tests/c/test_chassis_runtime.c"),
        str(FIRMWARE / "Core/Src/chassis_runtime.c"),
        str(FIRMWARE / "Core/Src/chassis_kinematics.c"),
        str(FIRMWARE / "Core/Src/dm2325_can.c"),
        "-lm", "-o", str(output),
    ]
    subprocess.run(command, check=True)
    subprocess.run([str(output)], check=True)
