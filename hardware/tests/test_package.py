import os
import subprocess
import sys
from pathlib import Path

HARDWARE = Path(__file__).resolve().parents[1]


def test_importing_neohw_leaves_no_skidl_logs_in_cwd(tmp_path):
    # `import skidl` opens <script>.log / <script>.erc in the working directory.
    env = {**os.environ, "PYTHONPATH": str(HARDWARE)}
    subprocess.run([sys.executable, "-c", "import skidl, neohw.parts"], cwd=tmp_path, env=env,
                   check=True, capture_output=True)
    assert os.listdir(tmp_path) == []
