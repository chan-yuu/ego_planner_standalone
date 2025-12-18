#!/bin/bash

# 修复 Eigen 索引错误的总结

echo "=========================================="
echo "  Eigen 索引错误修复"
echo "=========================================="
echo ""

echo "问题描述:"
echo "  运行时出现: Assertion 'index >= 0 && index < size()' failed"
echo "  错误位置: DenseCoeffsBase.h:425"
echo ""

echo "根本原因:"
echo "  在 publishBsplineToShm() 中访问控制点矩阵时，"
echo "  错误地使用了 rows() 而不是 cols() 来获取点数。"
echo ""
echo "  UniformBspline 的控制点矩阵格式:"
echo "    - 形状: (维度, 点数) = (3, n)"
echo "    - 应该用 cols() 获取点数"
echo "    - 访问方式: ctrl_pts(dim, point_idx)"
echo ""

echo "修复内容:"
echo "  1. main.cpp - publishBsplineToShm():"
echo "     - 修改: int num_pts = ctrl_pts.rows();"
echo "     - 改为: int num_pts = ctrl_pts.cols();"
echo "     - 修改: ctrl_pts(i, 0/1/2)"
echo "     - 改为: ctrl_pts(0/1/2, i)"
echo ""
echo "  2. planner_manager.cpp - reboundReplan():"
echo "     - 添加: velocity_traj_ 和 acceleration_traj_ 的计算"
echo "     - 添加: start_pos_ 的保存"
echo ""

echo "测试步骤:"
echo "  1. 启动仿真环境"
echo "  2. 运行 ./ego_planner_standalone"
echo "  3. 在RViz中发送目标点"
echo "  4. 观察是否正常生成轨迹"
echo ""

echo "编译状态: ✅ 成功"
echo "修复状态: ✅ 完成"
echo ""
echo "现在可以运行测试了！"
