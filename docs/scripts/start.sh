#!/bin/bash
# EGO Planner Standalone 快速启动脚本

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLANNER_DIR="$PROJECT_ROOT/planner_standalone/build"
EGO_DIR="$PROJECT_ROOT/ego-planner"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  EGO Planner Standalone 启动脚本${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检查文件是否存在
check_file() {
    if [ ! -f "$1" ]; then
        echo -e "${RED}错误: 找不到文件 $1${NC}"
        echo -e "${YELLOW}提示: 请先编译项目${NC}"
        exit 1
    fi
}

# 清理共享内存
clean_shm() {
    echo -e "${YELLOW}清理旧的共享内存段...${NC}"
    rm -f /dev/shm/ego_planner_* 2>/dev/null || true
    echo -e "${GREEN}✓ 共享内存已清理${NC}"
}

# 启动planner_standalone
start_planner() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  步骤1: 启动独立规划器${NC}"
    echo -e "${GREEN}========================================${NC}"
    
    check_file "$PLANNER_DIR/ego_planner_standalone"
    
    cd "$PLANNER_DIR"
    ./ego_planner_standalone
}

# 显示使用说明
show_usage() {
    echo -e "${GREEN}使用说明:${NC}"
    echo ""
    echo "1. 启动独立规划器 (本脚本):"
    echo -e "   ${YELLOW}./start.sh${NC}"
    echo ""
    echo "2. 在新终端启动ROS仿真环境:"
    echo -e "   ${YELLOW}cd ego-planner${NC}"
    echo -e "   ${YELLOW}source devel/setup.bash${NC}"
    echo -e "   ${YELLOW}roslaunch ego_planner_bridge sim_only.launch${NC}"
    echo ""
    echo "3. 在新终端启动ROS桥接:"
    echo -e "   ${YELLOW}cd ego-planner${NC}"
    echo -e "   ${YELLOW}source devel/setup.bash${NC}"
    echo -e "   ${YELLOW}roslaunch ego_planner_bridge run_bridge.launch${NC}"
    echo ""
    echo "4. 在RViz中使用 '2D Nav Goal' 工具发送目标点"
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo ""
}

# 主函数
main() {
    case "${1:-}" in
        --help|-h)
            show_usage
            ;;
        --clean|-c)
            clean_shm
            ;;
        *)
            clean_shm
            show_usage
            sleep 2
            start_planner
            ;;
    esac
}

main "$@"
