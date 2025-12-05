#!/bin/bash

echo "=== 测试新版本的规划器 ==="
cd /home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/planner_standalone/build

echo "启动规划器..."
./ego_planner_standalone 2>&1 | head -100
