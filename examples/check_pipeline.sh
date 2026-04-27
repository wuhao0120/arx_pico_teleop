#!/bin/bash
# ARX遥操数采Pipeline全面检查脚本
# 详细检查每个接口之间的交互细节
# Usage: ./check_pipeline.sh

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "====== ARX遥操数采Pipeline全面检查 ======"
echo ""

# 检查Python语法和基本导入
echo "====== 阶段1: 语法检查 ======"
cd "$PROJECT_ROOT"

# 检查核心文件的语法
echo "检查run_record_arx.py语法..."
python -m py_compile scripts/core/run_record_arx.py >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ run_record_arx.py 语法正确"
    rm -f __pycache__
else
    echo "  ❌ run_record_arx.py 语法错误"
    exit 1
fi

echo "检查teleop_arx.py语法..."
python -m py_compile teleoperators/arx/teleop_arx.py >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ teleop_arx.py 语法正确"
    rm -f __pycache__
else
    echo "  ❌ teleop_arx.py 语法错误"
    exit 1
fi

echo "检查config_teleop_arx.py语法..."
python -m py_compile teleoperators/arx/config_teleop_arx.py >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ config_teleop_arx.py 语法正确"
    rm -f __pycache__
else
    echo "  ❌ config_teleop_arx.py 语法错误"
    exit 1
fi

echo "检查arx_ros2_rpc_client.py语法..."
python -m py_compile ros2_bridge/arx_ros2_rpc_client.py >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ arx_ros2_rpc_client.py 语法正确"
    rm -f __pycache__
else
    echo "  ❌ arx_ros2_rpc_client.py 语法错误"
    exit 1
fi

echo "检查arx_ros2_rpc_server.py语法..."
python -m py_compile ros2_bridge/arx_ros2_rpc_server.py >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ arx_ros2_rpc_server.py 语法正确"
    rm -f __pycache__
else
    echo "  ❌ arx_ros2_rpc_server.py 语法错误"
    exit 1
fi

echo "检查arx_lift.py语法..."
python -m py_compile robots/arx/arx_lift.py >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ arx_lift.py 语法正确"
    rm -f __pycache__
else
    echo "  ❌ arx_lift.py 语法错误"
    exit 1
fi

echo "检查config_arx.py语法..."
python -m py_compile robots/arx/config_arx.py >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ config_arx.py 语法正确"
    rm -f __pycache__
else
    echo "  ❌ config_arx.py 语法错误"
    exit 1
fi

echo ""
echo "====== 阶段2: 配置文件检查 ======"

# 检查配置文件是否存在
CONFIG_FILE="scripts/config/cfg_arx.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

echo "✅ 配置文件存在: $CONFIG_FILE"

# 检查配置文件内容
echo "检查配置文件中的关键参数..."

# 检查repo_id
if grep -q "repo_id:" $CONFIG_FILE; then
    REPO_ID=$(grep 'repo_id:' $CONFIG_FILE | head -1 | awk '{print $2}')
    echo "  ✅ 仓库ID: $REPO_ID"
else
    echo "  ❌ 配置文件中缺少repo_id参数"
fi

# 检查debug模式
if grep -q "debug:" $CONFIG_FILE; then
    DEBUG=$(grep 'debug:' $CONFIG_FILE | head -1 | awk '{print $2}')
    echo "  ✅ Debug模式: $DEBUG"
else
    echo "  ❌ 配置文件中缺少debug参数"
fi

# 检查URDF路径
if grep -q "robot_urdf_path:" $CONFIG_FILE; then
    URDF_PATH=$(grep 'robot_urdf_path:' $CONFIG_FILE | head -1 | awk '{print $2}')
    FULL_PATH="${PROJECT_ROOT}/${URDF_PATH}"
    if [ -f "$FULL_PATH" ]; then
        echo "  ✅ URDF路径: $URDF_PATH"
    else
        echo "  ❌ URDF文件不存在: $FULL_PATH"
    fi
else
    echo "  ❌ 配置文件中缺少robot_urdf_path参数"
fi

# 检查相机配置
if grep -q "left_wrist_cam_serial:" $CONFIG_FILE; then
    LEFT_CAM=$(grep 'left_wrist_cam_serial:' $CONFIG_FILE | head -1 | awk '{print $2}')
    echo "  ✅ 左手腕相机序列号: $LEFT_CAM"
else
    echo "  ❌ 配置文件中缺少left_wrist_cam_serial参数"
fi

if grep -q "right_wrist_cam_serial:" $CONFIG_FILE; then
    RIGHT_CAM=$(grep 'right_wrist_cam_serial:' $CONFIG_FILE | head -1 | awk '{print $2}')
    echo "  ✅ 右手腕相机序列号: $RIGHT_CAM"
else
    echo "  ❌ 配置文件中缺少right_wrist_cam_serial参数"
fi

if grep -q "exterior_cam_serial:" $CONFIG_FILE; then
    EXTERIOR_CAM=$(grep 'exterior_cam_serial:' $CONFIG_FILE | head -1 | awk '{print $2}')
    echo "  ✅ 外部相机序列号: $EXTERIOR_CAM"
else
    echo "  ❌ 配置文件中缺少exterior_cam_serial参数"
fi

# 检查关节配置
if grep -q "left_init_joints:" $CONFIG_FILE; then
    LEFT_JOINTS=$(grep -A7 'left_init_joints:' $CONFIG_FILE | grep -o '[0-9.-]' | tr -d '\n')
    if [ ${#LEFT_JOINTS} -gt 0 ]; then
        echo "  ✅ 左臂初始关节配置存在"
    fi
else
    echo "  ❌ 配置文件中缺少left_init_joints参数"
fi

if grep -q "right_init_joints:" $CONFIG_FILE; then
    RIGHT_JOINTS=$(grep -A7 'right_init_joints:' $CONFIG_FILE | grep -o '[0-9.-]' | tr -d '\n')
    if [ ${#RIGHT_JOINTS} -gt 0 ]; then
        echo "  ✅ 右臂初始关节配置存在"
    fi
else
    echo "  ❌ 配置文件中缺少right_init_joints参数"
fi

echo ""
echo "====== 阶段3: 依赖检查 ======"

# 检查Python模块
echo "检查Python依赖模块..."

# 检查基本模块
for MODULE in "numpy" "scipy" "yaml" "pathlib" "logging" "threading" "signal"; do
    python -c "import $MODULE; print('✅ $MODULE')" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo "❌ 缺少依赖模块: $MODULE"
    fi
done

# 检查XRoboToolkit相关模块
echo "检查XRoboToolkit模块..."
python -c "import xrobotoolkit_teleop; print('✅ xrobotoolkit_teleop')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少xrobotoolkit_teleop模块"
    echo "   请运行: pip install -e ."
fi

# 检查LeRobot相关模块
echo "检查LeRobot模块..."
python -c "import lerobot; print('✅ lerobot')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少lerobot模块"
    echo "   请检查LeRobot安装"
fi

# 检查RealSense相关模块
echo "检查RealSense模块..."
python -c "import pyrealsense2; print('✅ pyrealsense2')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️  缺少pyrealsense2模块（可选）"
fi

# 检查 ZeroRPC 依赖模块
echo "检查ZeroRPC模块..."
python -c "import zerorpc; print('✅ zerorpc')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少zerorpc模块"
    echo "   请运行: pip install zerorpc"
fi

echo "检查gevent模块..."
python -c "import gevent; print('✅ gevent')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少gevent模块"
    echo "   请运行: pip install gevent"
fi

# 检查Placo模块
echo "检查Placo模块..."
python -c "import placo; print('✅ placo')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少placo模块"
    echo "   请运行: pip install placo"
fi

echo ""
echo "====== 阶段4: 端口和网络检查 ======"

# 检查RPC端口是否被占用
echo "检查RPC服务端口(4242)..."
if lsof -Pi :4242 -sTCP:LISTEN -t >/dev/null ; then
    echo "✅ RPC端口4242正在监听"
    PID=$(lsof -t -i:4242)
    echo "   PID: $PID"
else
    echo "⚠️  RPC服务未启动（将在运行时自动启动）"
fi

echo ""
echo "====== 阶段5: 文件权限检查 ======"

# 检查脚本执行权限
echo "检查脚本权限..."
if [ -x "scripts/core/run_record_arx.py" ]; then
    echo "✅ run_record_arx.py 执行权限: $(stat -c %a scripts/core/run_record_arx.py)"
else
    echo "⚠️  run_record_arx.py 缺少执行权限"
    chmod +x scripts/core/run_record_arx.py
    echo "   已添加执行权限"
fi

if [ -x "check_pipeline.sh" ]; then
    echo "✅ check_pipeline.sh 执行权限: $(stat -c %a check_pipeline.sh)"
else
    echo "⚠️  check_pipeline.sh 缺少执行权限"
    chmod +x check_pipeline.sh
    echo "   已添加执行权限"
fi

if [ -x "start_recording.sh" ]; then
    echo "✅ start_recording.sh 执行权限: $(stat -c %a start_recording.sh)"
else
    echo "⚠️  start_recording.sh 缺少执行权限"
    chmod +x start_recording.sh
    echo "   已添加执行权限"
fi

echo ""
echo "====== 阶段6: 文件存在性检查 ======"

echo "检查关键文件..."

# 检查assets目录
ASSETS_DIR="assets"
if [ -d "$ASSETS_DIR" ]; then
    echo "✅ Assets目录: $ASSETS_DIR"
    R5_URDF_DIR="assets/r5_urdf"
    if [ -d "$R5_URDF_DIR" ]; then
        echo "  ✅ R5 URDF目录: $R5_URDF_DIR"
        if [ -f "$R5_URDF_DIR/dual_R5a.urdf" ]; then
            echo "    ✅ 主URDF文件: dual_R5a.urdf"
        else
            echo "    ❌ 缺少URDF文件: dual_R5a.urdf"
        fi
    fi
else
    echo "❌ Assets目录不存在: $ASSETS_DIR"
fi

# 检查输出目录
OUTPUT_DIR="outputs"
if [ ! -d "$OUTPUT_DIR" ]; then
    mkdir -p "$OUTPUT_DIR"
    echo "✅ 创建输出目录: $OUTPUT_DIR"
else
    echo "✅ 输出目录: $OUTPUT_DIR"
fi

# 检查logs目录
LOGS_DIR="logs"
if [ ! -d "$LOGS_DIR" ]; then
    mkdir -p "$LOGS_DIR"
    echo "✅ 创建日志目录: $LOGS_DIR"
else
    echo "✅ 日志目录: $LOGS_DIR"
fi

echo ""
echo "====== 阶段7: ARX ROS2节点检查 ======"

echo "检查ROS2节点..."

# 检查是否有ROS2节点运行
echo "检查运行中的ROS2节点..."
if command -v ros2 &> /dev/null; then
    if ros2 node list 2>/dev/null >/dev/null; then
        NUM_NODES=$(ros2 node list | wc -l)
        echo "✅ ROS2运行正常，节点数: $NUM_NODES"
    else
        echo "⚠️  ROS2未启动（将在运行时自动启动）"
    fi
else
    echo "⚠️  ROS2命令不可用"
fi

echo ""
echo "====== 阶段8: 网络接口检查 ======"

echo "检查网络接口..."
for IFACE in "lo" "eth0" "wlan0"; do
    if ip link show $IFACE &> /dev/null; then
        STATE=$(ip link show $IFACE | grep -o "state [A-Z]*" | awk '{print $2}')
        echo "✅ $IFACE: $STATE"
    fi
done

echo ""
echo "====== 阶段9: 磁盘空间检查 ======"

echo "检查磁盘空间..."

# 检查项目所在磁盘的空间
DISK_USAGE=$(df -h "$PROJECT_ROOT" | tail -1 | awk '{print $5}' | sed 's/%//')
if [ "$DISK_USAGE" -lt 90 ]; then
    echo "✅ 磁盘空间充足 ($DISK_USAGE% 已使用)"
else
    echo "⚠️  磁盘空间紧张 ($DISK_USAGE% 已使用)"
fi

echo ""
echo "====== 阶段10: 最终检查总结 ======"

echo "✅ 所有基本检查完成！"
echo ""
echo "准备好运行了吗？"
echo ""
echo "选项1: 全自动启动（推荐）"
echo "  $ ./start_recording.sh debug   # 调试模式"
echo "  $ ./start_recording.sh record  # 正式录制"
echo ""
echo "选项2: 调试模式（详细输出，需手动启动控制器）"
echo "  $ python scripts/core/run_record_arx.py 2>&1 | tee logs/run_record_arx_$(date +%Y%m%d_%H%M%S).log"
echo ""
echo "选项3: 离线测试（无硬件）"
echo "  $ python test_teleop_arx_offline.py"
