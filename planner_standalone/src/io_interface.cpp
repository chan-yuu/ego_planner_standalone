/**
 * @file io_interface.cpp
 * @brief 输入输出接口工厂实现
 */

#include "io_interface/io_interface.h"
#include "io_interface/shm_io_interface.h"
#include <iostream>

namespace ego_planner {

std::unique_ptr<InputInterface> createInputInterface(const std::string& type) {
    if (type == "shm" || type == "shared_memory") {
        std::cout << "[IO Factory] 创建共享内存输入接口" << std::endl;
        return std::make_unique<ShmInputInterface>();
    }
    
    // 未来可以添加其他接口类型
    // else if (type == "socket") {
    //     return std::make_unique<SocketInputInterface>();
    // }
    // else if (type == "ros") {
    //     return std::make_unique<RosInputInterface>();
    // }
    
    std::cerr << "[IO Factory] 未知的输入接口类型: " << type << std::endl;
    return nullptr;
}

std::unique_ptr<OutputInterface> createOutputInterface(const std::string& type) {
    if (type == "shm" || type == "shared_memory") {
        std::cout << "[IO Factory] 创建共享内存输出接口" << std::endl;
        return std::make_unique<ShmOutputInterface>();
    }
    
    // 可以添加其他接口类型
    // else if (type == "socket") {
    //     return std::make_unique<SocketOutputInterface>();
    // }
    // else if (type == "ros") {
    //     return std::make_unique<RosOutputInterface>();
    // }
    
    std::cerr << "[IO Factory] 未知的输出接口类型: " << type << std::endl;
    return nullptr;
}

} // namespace ego_planner
