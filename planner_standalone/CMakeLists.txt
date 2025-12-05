cmake_minimum_required(VERSION 3.10)
project(ego_planner_standalone)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# 查找依赖
find_package(Eigen3 REQUIRED)

# 头文件目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# 源文件列表
set(PLANNER_SOURCES
    src/uniform_bspline.cpp
    src/bspline_optimizer.cpp
    src/gradient_descent_optimizer.cpp
    src/grid_map.cpp
    src/dyn_a_star.cpp
    src/polynomial_traj.cpp
    src/planner_manager.cpp
)

# I/O接口源文件
set(IO_INTERFACE_SOURCES
    src/io_interface.cpp
    src/shm_io_interface.cpp
)

# 创建库
add_library(${PROJECT_NAME}_lib STATIC ${PLANNER_SOURCES} ${IO_INTERFACE_SOURCES})
target_link_libraries(${PROJECT_NAME}_lib
    ${EIGEN3_LIBS}
    rt  # 共享内存需要
    pthread
)

# 创建可执行文件
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_lib
)

# 安装
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
