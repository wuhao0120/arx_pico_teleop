"""
一键启动命令入口点.

robot-rpc:    在机器人上位机启动 RPC server (CAN + 双臂控制器 + ZeroRPC)
robot-record: 在数采机启动 start_recording_remote_rpc (固定 record 模式)
"""

import os
import sys
from pathlib import Path


def _launch_shell_script(script_name: str, args: list[str] | None = None) -> None:
    """在包根目录查找并执行 shell 脚本."""
    package_root = Path(__file__).resolve().parent.parent
    script = package_root / script_name

    if not script.exists():
        print(f"[ERROR] 未找到启动脚本: {script}")
        sys.exit(1)

    script_args = args or []
    print(f"[INFO] 执行脚本: {script} {' '.join(script_args)}".rstrip())
    os.execvp("bash", ["bash", str(script), *script_args])


def launch_rpc_server() -> None:
    """robot-rpc 命令: 启动 start_rpc_server.sh.

    在机器人上位机上运行, 启动:
      1. CAN 接口初始化
      2. R5 双臂控制器 (ros2 launch)
      3. ZeroRPC 服务端 (tcp://0.0.0.0:4242)
    """
    _launch_shell_script("start_rpc_server.sh", sys.argv[1:])


def launch_remote_recording() -> None:
    """robot-record 命令: 启动 start_recording_remote_rpc.sh 的 record 模式.

    规则:
      1. 若 ARX_RPC_HOST 未设置且未传 host，默认使用 localhost。
      2. 无论用户是否传 debug/record，都会强制改为 record。
      3. 支持以下常见形式:
         - robot-record
         - robot-record <host>
         - robot-record <host> <port>
         - ARX_RPC_HOST=<host> robot-record [port]
    """
    cli_args = sys.argv[1:]
    host_from_env = os.environ.get("ARX_RPC_HOST")

    if host_from_env:
        # 已由环境变量给出 host，命令行仅接收 [mode] [port]，强制改为 record。
        if cli_args and cli_args[0] in {"debug", "record"}:
            cli_args = cli_args[1:]
        args = ["record", *cli_args]
    else:
        if not cli_args:
            os.environ["ARX_RPC_HOST"] = "localhost"
            args = ["record"]
        else:
            host = cli_args[0]
            rest = cli_args[1:]

            # 误把 mode 放在第一个参数时，回退到 localhost 并仍强制 record。
            if host in {"debug", "record"}:
                os.environ["ARX_RPC_HOST"] = "localhost"
                if rest and rest[0] in {"debug", "record"}:
                    rest = rest[1:]
                args = ["record", *rest]
            else:
                if rest and rest[0] in {"debug", "record"}:
                    rest = rest[1:]
                args = [host, "record", *rest]

    _launch_shell_script("start_recording_remote_rpc.sh", args)


def launch_remote_replay() -> None:
    """robot-replay 命令: 启动回放，通过 RPC 连接远程机器人执行。

    规则:
      1. 若 ARX_RPC_HOST 未设置且未传 host，报错提示。
      2. episode_idx 可选，指定后覆盖配置文件中的设置。
      3. 支持以下常见形式:
         - robot-replay                     (ARX_RPC_HOST 已在环境中设置)
         - robot-replay <episode_idx>      (ARX_RPC_HOST 已在环境中设置)
         - robot-replay <host> <episode_idx>
         - ARX_RPC_HOST=<host> robot-replay
    """
    cli_args = sys.argv[1:]
    host_from_env = os.environ.get("ARX_RPC_HOST")
    args = cli_args

    if not host_from_env and (not cli_args or (cli_args and cli_args[0].isnumeric())):
        print("[ERROR] 未设置 ARX_RPC_HOST，且第一个参数不是主机地址。")
        print("用法: robot-replay <host> [episode_idx]")
        print("  或: export ARX_RPC_HOST=<host> && robot-replay [episode_idx]")
        sys.exit(1)

    _launch_shell_script("start_replay_remote_rpc.sh", args)


def launch_robot_reset() -> None:
    """robot-reset 命令: 移动双臂到初始（home/reset）位置。

    通过 RPC 连接远程机器人，读取配置文件中的 left_init_joints / right_init_joints，
    发送命令让双臂移动到该位置。

    用法:
      robot-reset
      ARX_RPC_HOST=<host> robot-reset
    """
    import sys
    cli_args = sys.argv[1:]
    host_from_env = os.environ.get("ARX_RPC_HOST")

    if not host_from_env:
        if cli_args and not cli_args[0].isnumeric():
            os.environ["ARX_RPC_HOST"] = cli_args[0]
        else:
            print("[ERROR] 未设置 ARX_RPC_HOST，也没有在命令行给出主机地址。")
            print("用法: robot-reset <host>")
            print("  或: export ARX_RPC_HOST=<host> && robot-reset")
            sys.exit(1)

    # Reuse the robot reset script directly
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from scripts.robot_reset import main
    main()
