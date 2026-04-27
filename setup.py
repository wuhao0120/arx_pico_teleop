from setuptools import setup, find_packages
from pathlib import Path
# ====== Project root ======
ROOT = Path(__file__).parent.resolve()
setup(
    name="arx_pico_teleop",
    version="0.1.0",
    description="Pico 4 VR teleoperation and dataset collection for ARX robots",
    python_requires=">=3.10",
    packages=find_packages(where=".", include=[
        "scripts", "scripts.*",
        "robots", "robots.*",
        "teleoperators", "teleoperators.*",
        "xrobotoolkit_teleop", "xrobotoolkit_teleop.*",
    ]),
    include_package_data=True,
    install_requires=[
    "send2trash",
    "zerorpc"
    ],
    scripts=[
        "scripts/tools/map_gripper.sh",
    ],
    entry_points={
        "console_scripts": [
            # === 主要命令 (pico 遥操) ===
            "pico-record = scripts.launcher:launch_remote_recording",  # Client 端: 远程 RPC 数采(record 模式)
            "pico-replay = scripts.launcher:launch_remote_replay",      # Client 端: 远程 RPC 回放
            "pico-reset = scripts.launcher:launch_robot_reset",          # Client 端: 移动双臂到初始位置
            "pico-rpc = scripts.launcher:launch_rpc_server",            # Server 端: 启动 RPC server

            # === 原有 UR 命令 (保留兼容) ===
            "vr-record = scripts.core.run_record:main",
            "vr-replay = scripts.core.run_replay:main",
            "vr-visualize = scripts.core.run_visualize:main",
            "arx-record = scripts.core.run_record_arx:main",

            # === 工具 ===
            "tools-check-dataset = scripts.tools.check_dataset_info:main",
            "tools-check-rs = scripts.tools.rs_devices:main",

            # === 测试 ===
            "test-gripper-ctrl = scripts.tests.gripper_ctrl:main",
            "test-ur-freedrive = scripts.tests.ur_freedrive:main",
            "test-arx-connection = scripts.tests.test_arx_modules:main",

            # === 帮助 ===
            "vr-help = scripts.help.help_info:main",
        ]
    },
)
