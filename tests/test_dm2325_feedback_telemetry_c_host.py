import subprocess
from pathlib import Path


def test_dm2325_feedback_telemetry_host(tmp_path):
    root = Path(__file__).resolve().parents[1]
    firmware = root / "firmware" / "robomaster_c_xelerator_t2"
    output = tmp_path / "dm2325_feedback_telemetry"
    subprocess.run([
        "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        f"-I{firmware / 'Core' / 'Inc'}",
        str(root / "tests" / "c" / "test_dm2325_feedback_telemetry.c"),
        str(firmware / "Core" / "Src" / "dm2325_can.c"),
        "-lm", "-o", str(output),
    ], check=True)
    subprocess.run([str(output)], check=True)
