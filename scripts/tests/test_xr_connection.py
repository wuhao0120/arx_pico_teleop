#!/usr/bin/env python3
"""
测试 XRoboToolkit VR 连接

这个脚本测试 VR 头显（PICO 4 Ultra / Meta Quest）与电脑的连接。
在使用 VR 遥操之前，请先确保此测试通过。

前置条件：
1. 确保 XRoboToolkit PC Service 正在运行
2. VR 头显与电脑在同一 WiFi 网络
3. VR 头显上的 XRoboToolkit 应用已启动

使用方法：
    python scripts/tools/test_xr_connection.py
"""

import sys
import time
import numpy as np

# Add XRoboToolkit path (in lerobot_data_collection directory)
import os
_script_dir = os.path.dirname(os.path.abspath(__file__))
_scripts_dir = os.path.dirname(_script_dir)
_project_root = os.path.dirname(_scripts_dir)
_lerobot_data_root = os.path.dirname(_project_root)
_xr_toolkit_path = os.path.join(_lerobot_data_root, "xrobotoolkit_teleop")
sys.path.insert(0, _xr_toolkit_path)

np.set_printoptions(suppress=True, precision=3)


def test_sdk_import():
    """测试 1: SDK 导入"""
    print("\n" + "=" * 60)
    print("测试 1: XRoboToolkit SDK 导入")
    print("=" * 60)

    try:
        import xrobotoolkit_sdk as xrt
        print("✓ xrobotoolkit_sdk 导入成功")
        return True, xrt
    except ImportError as e:
        print(f"❌ 导入失败: {e}")
        print("")
        print("请确保 XRoboToolkit 已正确安装：")
        print(f"  cd {_xr_toolkit_path}")
        print("  bash setup_conda.sh --install")
        return False, None


def test_sdk_init(xrt):
    """测试 2: SDK 初始化"""
    print("\n" + "=" * 60)
    print("测试 2: SDK 初始化")
    print("=" * 60)

    try:
        xrt.init()
        print("✓ SDK 初始化成功")
        return True
    except Exception as e:
        print(f"❌ 初始化失败: {e}")
        print("")
        print("请确保 XRoboToolkit PC Service 正在运行：")
        print("  1. 下载安装 XRoboToolkit PC Service")
        print("     https://github.com/XR-Robotics/XRoboToolkit-PC-Service")
        print("  2. 启动服务后再运行此测试")
        return False


def interactive_test_buttons(xrt):
    """交互式测试: 所有按钮"""
    print("\n" + "=" * 60)
    print("交互式测试: VR 控制器按钮")
    print("=" * 60)
    print("")
    print("请按照提示依次按下 VR 控制器上的按钮。")
    print("每个按钮有 10 秒超时。按 Ctrl+C 跳过当前测试。")
    print("")

    # 所有需要测试的按钮
    buttons = [
        ("A", xrt.get_A_button, "右手柄 A 按钮"),
        ("B", xrt.get_B_button, "右手柄 B 按钮"),
        ("X", xrt.get_X_button, "左手柄 X 按钮"),
        ("Y", xrt.get_Y_button, "左手柄 Y 按钮"),
        ("左菜单键", xrt.get_left_menu_button, "左手柄 Menu 按钮"),
        ("右菜单键", xrt.get_right_menu_button, "右手柄 Menu 按钮"),
        ("左摇杆按下", xrt.get_left_axis_click, "左手柄摇杆按下"),
        ("右摇杆按下", xrt.get_right_axis_click, "右手柄摇杆按下"),
    ]

    results = {}

    for name, get_func, description in buttons:
        print(f"📌 请按下 {description}...")

        try:
            waited = 0
            detected = False
            while waited < 10:
                if get_func():
                    print(f"   ✓ 检测到 {name} 按钮按下！")
                    detected = True
                    time.sleep(0.3)  # 等待松开
                    break
                time.sleep(0.1)
                waited += 0.1

            if not detected:
                print(f"   ⏱️ 超时，未检测到 {name}")

            results[name] = detected

        except KeyboardInterrupt:
            print(f"   ⏭️ 跳过 {name}")
            results[name] = None

    # 测试扳机和握把（模拟值）
    print("")
    print("📌 测试扳机和握把（模拟值）")
    print("   请分别按下左右扳机和握把...")

    try:
        time.sleep(0.5)
        triggers = [
            ("左扳机", xrt.get_left_trigger),
            ("右扳机", xrt.get_right_trigger),
            ("左握把", xrt.get_left_grip),
            ("右握把", xrt.get_right_grip),
        ]

        for name, get_func in triggers:
            print(f"   请按下 {name}...")
            max_val = 0
            for _ in range(30):  # 3秒
                val = get_func()
                max_val = max(max_val, val)
                if val > 0.5:
                    break
                time.sleep(0.1)

            if max_val > 0.5:
                print(f"   ✓ {name}: 检测到按下 (最大值: {max_val:.2f})")
                results[name] = True
            else:
                print(f"   ⚠️ {name}: 未检测到明显按下 (最大值: {max_val:.2f})")
                results[name] = False

    except KeyboardInterrupt:
        print("   ⏭️ 跳过扳机/握把测试")

    # 总结
    print("")
    print("按钮测试结果:")
    passed = sum(1 for v in results.values() if v is True)
    total = len(results)
    for name, result in results.items():
        if result is True:
            status = "✓ 通过"
        elif result is False:
            status = "❌ 失败"
        else:
            status = "⏭️ 跳过"
        print(f"   {status}: {name}")

    print(f"\n通过: {passed}/{total}")
    return passed >= total * 0.7  # 70% 通过即可


def interactive_test_motion(xrt):
    """交互式测试: 手部和头部运动数据"""
    print("\n" + "=" * 60)
    print("交互式测试: 运动追踪数据")
    print("=" * 60)
    print("")
    print("这个测试验证手柄和头显在三维空间中的位置追踪。")
    print("")

    input("按 Enter 开始运动测试...")
    print("")

    # 测试 1: 左手柄运动
    print("📌 测试 1: 左手柄运动追踪")
    print("   请拿起左手柄，在空中画一个小圈（约 5 秒）...")
    print("")

    try:
        left_positions = []
        for i in range(50):  # 5秒，10Hz
            pose = xrt.get_left_controller_pose()
            left_positions.append(pose[:3])  # xyz
            time.sleep(0.1)

        left_positions = np.array(left_positions)
        left_range = np.ptp(left_positions, axis=0)  # 每个轴的范围
        left_total_range = np.linalg.norm(left_range)

        print(f"   左手柄位置范围:")
        print(f"      X: {left_range[0]:.3f} m")
        print(f"      Y: {left_range[1]:.3f} m")
        print(f"      Z: {left_range[2]:.3f} m")
        print(f"      总范围: {left_total_range:.3f} m")

        if left_total_range > 0.05:  # 移动了至少 5cm
            print("   ✓ 左手柄运动追踪正常！")
            left_ok = True
        else:
            print("   ⚠️ 左手柄移动范围较小，可能追踪有问题")
            left_ok = False

    except KeyboardInterrupt:
        print("   ⏭️ 跳过")
        left_ok = None

    print("")

    # 测试 2: 右手柄运动
    print("📌 测试 2: 右手柄运动追踪")
    print("   请拿起右手柄，在空中画一个小圈（约 5 秒）...")
    print("")

    try:
        right_positions = []
        for i in range(50):
            pose = xrt.get_right_controller_pose()
            right_positions.append(pose[:3])
            time.sleep(0.1)

        right_positions = np.array(right_positions)
        right_range = np.ptp(right_positions, axis=0)
        right_total_range = np.linalg.norm(right_range)

        print(f"   右手柄位置范围:")
        print(f"      X: {right_range[0]:.3f} m")
        print(f"      Y: {right_range[1]:.3f} m")
        print(f"      Z: {right_range[2]:.3f} m")
        print(f"      总范围: {right_total_range:.3f} m")

        if right_total_range > 0.05:
            print("   ✓ 右手柄运动追踪正常！")
            right_ok = True
        else:
            print("   ⚠️ 右手柄移动范围较小，可能追踪有问题")
            right_ok = False

    except KeyboardInterrupt:
        print("   ⏭️ 跳过")
        right_ok = None

    print("")

    # 测试 3: 头显运动
    print("📌 测试 3: 头显运动追踪")
    print("   请戴着头显，转动头部左右看（约 5 秒）...")
    print("")

    try:
        head_positions = []
        head_orientations = []
        for i in range(50):
            pose = xrt.get_headset_pose()
            head_positions.append(pose[:3])
            head_orientations.append(pose[3:])  # quaternion
            time.sleep(0.1)

        head_positions = np.array(head_positions)
        head_orientations = np.array(head_orientations)

        head_pos_range = np.ptp(head_positions, axis=0)
        head_quat_range = np.ptp(head_orientations, axis=0)

        print(f"   头显位置变化范围:")
        print(f"      X: {head_pos_range[0]:.3f} m")
        print(f"      Y: {head_pos_range[1]:.3f} m")
        print(f"      Z: {head_pos_range[2]:.3f} m")
        print(f"   头显姿态变化范围 (quaternion):")
        print(f"      qx: {head_quat_range[0]:.3f}")
        print(f"      qy: {head_quat_range[1]:.3f}")
        print(f"      qz: {head_quat_range[2]:.3f}")
        print(f"      qw: {head_quat_range[3]:.3f}")

        # 头部旋转主要体现在 quaternion 变化上
        head_rotation_change = np.linalg.norm(head_quat_range)
        if head_rotation_change > 0.1:
            print("   ✓ 头显运动追踪正常！")
            head_ok = True
        else:
            print("   ⚠️ 头显姿态变化较小，可能追踪有问题")
            head_ok = False

    except KeyboardInterrupt:
        print("   ⏭️ 跳过")
        head_ok = None

    print("")

    # 测试 4: 摇杆
    print("📌 测试 4: 摇杆输入")
    print("   请推动左右摇杆（约 3 秒）...")
    print("")

    try:
        left_joy_range = [0, 0]
        right_joy_range = [0, 0]

        for i in range(30):
            left_joy = xrt.get_left_axis()
            right_joy = xrt.get_right_axis()

            left_joy_range[0] = max(left_joy_range[0], abs(left_joy[0]))
            left_joy_range[1] = max(left_joy_range[1], abs(left_joy[1]))
            right_joy_range[0] = max(right_joy_range[0], abs(right_joy[0]))
            right_joy_range[1] = max(right_joy_range[1], abs(right_joy[1]))

            time.sleep(0.1)

        print(f"   左摇杆最大值: X={left_joy_range[0]:.2f}, Y={left_joy_range[1]:.2f}")
        print(f"   右摇杆最大值: X={right_joy_range[0]:.2f}, Y={right_joy_range[1]:.2f}")

        joy_ok = max(left_joy_range + right_joy_range) > 0.3
        if joy_ok:
            print("   ✓ 摇杆输入正常！")
        else:
            print("   ⚠️ 摇杆输入较弱")

    except KeyboardInterrupt:
        print("   ⏭️ 跳过")
        joy_ok = None

    # 总结
    print("")
    print("-" * 40)
    print("运动追踪测试结果:")
    results = {
        "左手柄追踪": left_ok,
        "右手柄追踪": right_ok,
        "头显追踪": head_ok,
        "摇杆输入": joy_ok,
    }

    for name, result in results.items():
        if result is True:
            status = "✓ 通过"
        elif result is False:
            status = "❌ 失败"
        else:
            status = "⏭️ 跳过"
        print(f"   {status}: {name}")

    passed = sum(1 for v in results.values() if v is True)
    total = sum(1 for v in results.values() if v is not None)

    return passed >= total * 0.7 if total > 0 else True


def realtime_monitor(xrt, duration=10):
    """实时监控模式：显示所有输入数据"""
    print("\n" + "=" * 60)
    print("实时监控模式")
    print("=" * 60)
    print("")
    print(f"显示 {duration} 秒的实时数据，按 Ctrl+C 提前退出...")
    print("")

    try:
        start_time = time.time()
        while time.time() - start_time < duration:
            # 清屏效果（打印足够多的换行）
            print("\033[H\033[J", end="")  # ANSI 清屏

            print("=" * 60)
            print("实时 VR 数据监控 (按 Ctrl+C 退出)")
            print("=" * 60)

            # 头显
            head = xrt.get_headset_pose()
            print(f"\n头显:")
            print(f"  位置: x={head[0]:7.3f}, y={head[1]:7.3f}, z={head[2]:7.3f}")
            print(f"  姿态: qx={head[3]:6.3f}, qy={head[4]:6.3f}, qz={head[5]:6.3f}, qw={head[6]:6.3f}")

            # 左手柄
            left = xrt.get_left_controller_pose()
            print(f"\n左手柄:")
            print(f"  位置: x={left[0]:7.3f}, y={left[1]:7.3f}, z={left[2]:7.3f}")
            print(f"  姿态: qx={left[3]:6.3f}, qy={left[4]:6.3f}, qz={left[5]:6.3f}, qw={left[6]:6.3f}")
            print(f"  扳机: {xrt.get_left_trigger():5.2f}  握把: {xrt.get_left_grip():5.2f}")

            # 右手柄
            right = xrt.get_right_controller_pose()
            print(f"\n右手柄:")
            print(f"  位置: x={right[0]:7.3f}, y={right[1]:7.3f}, z={right[2]:7.3f}")
            print(f"  姿态: qx={right[3]:6.3f}, qy={right[4]:6.3f}, qz={right[5]:6.3f}, qw={right[6]:6.3f}")
            print(f"  扳机: {xrt.get_right_trigger():5.2f}  握把: {xrt.get_right_grip():5.2f}")

            # 摇杆
            left_joy = xrt.get_left_axis()
            right_joy = xrt.get_right_axis()
            print(f"\n摇杆:")
            print(f"  左: x={left_joy[0]:6.2f}, y={left_joy[1]:6.2f}")
            print(f"  右: x={right_joy[0]:6.2f}, y={right_joy[1]:6.2f}")

            # 按钮
            buttons = []
            if xrt.get_A_button(): buttons.append("A")
            if xrt.get_B_button(): buttons.append("B")
            if xrt.get_X_button(): buttons.append("X")
            if xrt.get_Y_button(): buttons.append("Y")
            if xrt.get_left_menu_button(): buttons.append("L-Menu")
            if xrt.get_right_menu_button(): buttons.append("R-Menu")

            print(f"\n按下的按钮: {', '.join(buttons) if buttons else '(无)'}")

            elapsed = time.time() - start_time
            print(f"\n剩余时间: {duration - elapsed:.1f} 秒")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n已退出实时监控")

    return True


def cleanup(xrt):
    """清理资源"""
    try:
        xrt.close()
    except:
        pass


def main():
    print("=" * 60)
    print("XRoboToolkit VR 连接测试")
    print("=" * 60)
    print("")
    print("此测试适用于：PICO 4 Ultra / Meta Quest3 等 VR 设备")
    print("")

    results = {}

    # 测试 1: 导入
    success, xrt = test_sdk_import()
    results['SDK 导入'] = success

    if not success:
        print("\n❌ 无法继续测试，请先修复导入问题")
        return 1

    # 测试 2: 初始化
    results['SDK 初始化'] = test_sdk_init(xrt)

    if not results['SDK 初始化']:
        print("\n❌ 无法继续测试，请先启动 XRoboToolkit PC Service")
        return 1

    # 选择测试模式
    print("\n" + "=" * 60)
    print("选择测试模式")
    print("=" * 60)
    print("")
    print("  1. 按钮测试 - 测试所有按钮输入")
    print("  2. 运动测试 - 测试手柄和头显追踪")
    print("  3. 实时监控 - 显示实时数据流")
    print("  4. 全部测试 - 执行所有测试")
    print("  5. 退出")
    print("")

    try:
        choice = input("请选择 (1-5) [默认: 4]: ").strip() or "4"
    except KeyboardInterrupt:
        choice = "5"

    if choice == "1":
        results['按钮测试'] = interactive_test_buttons(xrt)
    elif choice == "2":
        results['运动测试'] = interactive_test_motion(xrt)
    elif choice == "3":
        results['实时监控'] = realtime_monitor(xrt, duration=30)
    elif choice == "4":
        results['按钮测试'] = interactive_test_buttons(xrt)
        results['运动测试'] = interactive_test_motion(xrt)
    elif choice == "5":
        print("退出测试")
        cleanup(xrt)
        return 0

    # 清理
    cleanup(xrt)

    # 总结
    print("\n" + "=" * 60)
    print("测试总结")
    print("=" * 60)

    passed = sum(1 for v in results.values() if v is True)
    total = len(results)

    for name, result in results.items():
        if result is True:
            status = "✓ 通过"
        elif result is False:
            status = "❌ 失败"
        else:
            status = "⏭️ 跳过"
        print(f"  {status}: {name}")

    print(f"\n结果: {passed}/{total} 测试通过")

    if passed == total:
        print("\n✓ VR 连接正常！可以开始 VR 遥操。")
        return 0
    else:
        print("\n⚠️ 部分测试未通过，请检查上述问题。")
        return 1


if __name__ == "__main__":
    sys.exit(main())
