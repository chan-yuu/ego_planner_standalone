#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EGO-Planner 控制面板
可视化界面用于编译、启动和管理各个模块
"""

import os
import sys
import subprocess
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext
from pathlib import Path
import signal
import time

class EGOPlannerControlPanel:
    def __init__(self, root):
        self.root = root
        self.root.title("EGO-Planner 控制面板")
        self.root.geometry("1000x700")
        
        # 获取项目根目录
        self.project_root = Path(__file__).resolve().parent
        self.ego_planner_path = self.project_root / "ego-planner"
        self.planner_standalone_path = self.project_root / "planner_standalone"
        
        # 进程字典
        self.processes = {}
        
        # 创建界面
        self.create_widgets()
        
        # 设置关闭回调
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_widgets(self):
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 标题
        title_label = ttk.Label(main_frame, text="EGO-Planner 控制面板", 
                                font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=10)
        
        # ========== 编译区域 ==========
        compile_frame = ttk.LabelFrame(main_frame, text="编译模块", padding="10")
        compile_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # 编译 EGO-Planner
        self.btn_compile_ego = ttk.Button(compile_frame, text="编译 EGO-Planner (ROS)",
                                          command=self.compile_ego_planner, width=25)
        self.btn_compile_ego.grid(row=0, column=0, padx=5, pady=5)
        self.lbl_ego_status = ttk.Label(compile_frame, text="未编译", foreground="gray")
        self.lbl_ego_status.grid(row=0, column=1, padx=5)
        
        # 编译 Planner Standalone
        self.btn_compile_standalone = ttk.Button(compile_frame, text="编译 独立规划器",
                                                 command=self.compile_planner_standalone, width=25)
        self.btn_compile_standalone.grid(row=1, column=0, padx=5, pady=5)
        self.lbl_standalone_status = ttk.Label(compile_frame, text="未编译", foreground="gray")
        self.lbl_standalone_status.grid(row=1, column=1, padx=5)
        
        # ========== 启动区域 ==========
        launch_frame = ttk.LabelFrame(main_frame, text="启动模块", padding="10")
        launch_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # 第一行：仿真环境
        self.btn_sim = ttk.Button(launch_frame, text="启动仿真环境",
                                  command=lambda: self.launch_roslaunch("sim", 
                                      "ego_planner_bridge", "sim_only.launch"),
                                  width=25)
        self.btn_sim.grid(row=0, column=0, padx=5, pady=5)
        self.lbl_sim_status = ttk.Label(launch_frame, text="●", foreground="red")
        self.lbl_sim_status.grid(row=0, column=1, padx=5)
        self.btn_stop_sim = ttk.Button(launch_frame, text="停止", 
                                       command=lambda: self.stop_process("sim"),
                                       width=10, state=tk.DISABLED)
        self.btn_stop_sim.grid(row=0, column=2, padx=5)
        
        # 第二行：Grid Map
        self.btn_gridmap = ttk.Button(launch_frame, text="启动 Grid Map",
                                      command=lambda: self.launch_roslaunch("gridmap",
                                          "grid_map_standalone", "test_grid_map.launch"),
                                      width=25)
        self.btn_gridmap.grid(row=1, column=0, padx=5, pady=5)
        self.lbl_gridmap_status = ttk.Label(launch_frame, text="●", foreground="red")
        self.lbl_gridmap_status.grid(row=1, column=1, padx=5)
        self.btn_stop_gridmap = ttk.Button(launch_frame, text="停止",
                                           command=lambda: self.stop_process("gridmap"),
                                           width=10, state=tk.DISABLED)
        self.btn_stop_gridmap.grid(row=1, column=2, padx=5)
        
        # 第三行：ROS Bridge
        self.btn_bridge = ttk.Button(launch_frame, text="启动 ROS Bridge",
                                     command=lambda: self.launch_roslaunch("bridge",
                                         "ego_planner_bridge", "run_bridge.launch"),
                                     width=25)
        self.btn_bridge.grid(row=2, column=0, padx=5, pady=5)
        self.lbl_bridge_status = ttk.Label(launch_frame, text="●", foreground="red")
        self.lbl_bridge_status.grid(row=2, column=1, padx=5)
        self.btn_stop_bridge = ttk.Button(launch_frame, text="停止",
                                          command=lambda: self.stop_process("bridge"),
                                          width=10, state=tk.DISABLED)
        self.btn_stop_bridge.grid(row=2, column=2, padx=5)
        
        # 第四行：独立规划器
        self.btn_planner = ttk.Button(launch_frame, text="启动 独立规划器",
                                      command=self.launch_planner_standalone,
                                      width=25)
        self.btn_planner.grid(row=3, column=0, padx=5, pady=5)
        self.lbl_planner_status = ttk.Label(launch_frame, text="●", foreground="red")
        self.lbl_planner_status.grid(row=3, column=1, padx=5)
        self.btn_stop_planner = ttk.Button(launch_frame, text="停止",
                                           command=lambda: self.stop_process("planner"),
                                           width=10, state=tk.DISABLED)
        self.btn_stop_planner.grid(row=3, column=2, padx=5)
        
        # ========== 快捷操作 ==========
        quick_frame = ttk.LabelFrame(main_frame, text="快捷操作", padding="10")
        quick_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        self.btn_start_all = ttk.Button(quick_frame, text="启动全部",
                                        command=self.start_all, width=20)
        self.btn_start_all.grid(row=0, column=0, padx=5, pady=5)
        
        self.btn_stop_all = ttk.Button(quick_frame, text="停止全部",
                                       command=self.stop_all, width=20)
        self.btn_stop_all.grid(row=0, column=1, padx=5, pady=5)
        
        # ========== 日志输出 ==========
        log_frame = ttk.LabelFrame(main_frame, text="日志输出", padding="10")
        log_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=20, width=100)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 清空日志按钮
        self.btn_clear_log = ttk.Button(log_frame, text="清空日志",
                                        command=self.clear_log, width=15)
        self.btn_clear_log.grid(row=1, column=0, pady=5)
        
        # ========== 底部作者信息 ==========
        author_frame = ttk.Frame(main_frame)
        author_frame.grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        
        author_label = ttk.Label(author_frame, text="© 2025 cyun - EGO-PLANNER Control Panel", 
                                 font=("Arial", 9), foreground="gray")
        author_label.pack()
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(4, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
    
    def log(self, message, level="INFO"):
        """输出日志"""
        timestamp = time.strftime("%H:%M:%S")
        colors = {
            "INFO": "black",
            "SUCCESS": "green",
            "ERROR": "red",
            "WARNING": "orange"
        }
        
        self.log_text.insert(tk.END, f"[{timestamp}] [{level}] {message}\n")
        self.log_text.tag_config(level, foreground=colors.get(level, "black"))
        self.log_text.see(tk.END)
        self.root.update()
    
    def clear_log(self):
        """清空日志"""
        self.log_text.delete(1.0, tk.END)
    
    def run_command(self, cmd, cwd=None, shell=True, capture_output=True):
        """运行命令并捕获输出"""
        try:
            self.log(f"执行命令: {cmd}")
            
            if capture_output:
                result = subprocess.run(cmd, shell=shell, cwd=cwd, 
                                       capture_output=True, text=True, timeout=300,
                                       executable='/bin/bash')
                
                if result.stdout:
                    self.log(result.stdout)
                if result.stderr:
                    self.log(result.stderr, "WARNING")
                
                if result.returncode == 0:
                    self.log("命令执行成功", "SUCCESS")
                    return True
                else:
                    self.log(f"命令执行失败 (返回码: {result.returncode})", "ERROR")
                    return False
            else:
                # 不捕获输出，直接启动 - 使用bash执行
                process = subprocess.Popen(cmd, shell=shell, cwd=cwd,
                                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                          executable='/bin/bash',
                                          preexec_fn=os.setsid)
                return process
                
        except subprocess.TimeoutExpired:
            self.log("命令执行超时", "ERROR")
            return False
        except Exception as e:
            self.log(f"执行命令时出错: {str(e)}", "ERROR")
            return False
    
    def compile_ego_planner(self):
        """编译 EGO-Planner"""
        self.btn_compile_ego.config(state=tk.DISABLED)
        self.lbl_ego_status.config(text="编译中...", foreground="orange")
        
        def compile_thread():
            self.log("开始编译 EGO-Planner (ROS)...")
            
            # 使用bash执行source命令
            cmd = f"source /opt/ros/noetic/setup.bash && " \
                  f"cd {self.ego_planner_path} && " \
                  f"catkin_make"
            
            success = self.run_command(cmd)
            
            if success:
                self.lbl_ego_status.config(text="编译成功", foreground="green")
                self.log("EGO-Planner 编译完成", "SUCCESS")
            else:
                self.lbl_ego_status.config(text="编译失败", foreground="red")
                self.log("EGO-Planner 编译失败", "ERROR")
            
            self.btn_compile_ego.config(state=tk.NORMAL)
        
        thread = threading.Thread(target=compile_thread, daemon=True)
        thread.start()
    
    def compile_planner_standalone(self):
        """编译独立规划器"""
        self.btn_compile_standalone.config(state=tk.DISABLED)
        self.lbl_standalone_status.config(text="编译中...", foreground="orange")
        
        def compile_thread():
            self.log("开始编译独立规划器...")
            
            build_dir = self.planner_standalone_path / "build"
            
            # 创建 build 目录（如果不存在）
            if not build_dir.exists():
                build_dir.mkdir(parents=True)
                self.log("创建 build 目录")
            
            # CMake 配置
            cmd_cmake = f"cd {build_dir} && cmake .."
            success = self.run_command(cmd_cmake)
            
            if not success:
                self.lbl_standalone_status.config(text="配置失败", foreground="red")
                self.log("CMake 配置失败", "ERROR")
                self.btn_compile_standalone.config(state=tk.NORMAL)
                return
            
            # Make 编译
            cmd_make = f"cd {build_dir} && make -j4"
            success = self.run_command(cmd_make)
            
            if success:
                self.lbl_standalone_status.config(text="编译成功", foreground="green")
                self.log("独立规划器编译完成", "SUCCESS")
            else:
                self.lbl_standalone_status.config(text="编译失败", foreground="red")
                self.log("独立规划器编译失败", "ERROR")
            
            self.btn_compile_standalone.config(state=tk.NORMAL)
        
        thread = threading.Thread(target=compile_thread, daemon=True)
        thread.start()
    
    def launch_roslaunch(self, process_name, package, launch_file):
        """启动 roslaunch"""
        if process_name in self.processes:
            self.log(f"{process_name} 已经在运行", "WARNING")
            return
        
        def launch_thread():
            self.log(f"启动 {package} {launch_file}...")
            
            # Source workspace
            cmd = f"source {self.ego_planner_path}/devel/setup.bash && " \
                  f"roslaunch {package} {launch_file}"
            
            process = self.run_command(cmd, capture_output=False)
            
            if process:
                self.processes[process_name] = process
                self.update_status(process_name, True)
                self.log(f"{process_name} 启动成功", "SUCCESS")
                
                # 等待进程结束
                process.wait()
                
                # 进程结束后清理
                if process_name in self.processes:
                    del self.processes[process_name]
                    self.update_status(process_name, False)
                    self.log(f"{process_name} 已停止")
            else:
                self.log(f"{process_name} 启动失败", "ERROR")
        
        thread = threading.Thread(target=launch_thread, daemon=True)
        thread.start()
    
    def launch_planner_standalone(self):
        """启动独立规划器"""
        if "planner" in self.processes:
            self.log("独立规划器已经在运行", "WARNING")
            return
        
        def launch_thread():
            self.log("启动独立规划器...")
            
            executable = self.planner_standalone_path / "build" / "ego_planner_standalone"
            
            if not executable.exists():
                self.log("可执行文件不存在，请先编译", "ERROR")
                return
            
            cmd = str(executable)
            process = self.run_command(cmd, cwd=str(self.planner_standalone_path / "build"),
                                      capture_output=False)
            
            if process:
                self.processes["planner"] = process
                self.update_status("planner", True)
                self.log("独立规划器启动成功", "SUCCESS")
                
                # 等待进程结束
                process.wait()
                
                # 进程结束后清理
                if "planner" in self.processes:
                    del self.processes["planner"]
                    self.update_status("planner", False)
                    self.log("独立规划器已停止")
            else:
                self.log("独立规划器启动失败", "ERROR")
        
        thread = threading.Thread(target=launch_thread, daemon=True)
        thread.start()
    
    def stop_process(self, process_name):
        """停止进程"""
        if process_name not in self.processes:
            self.log(f"{process_name} 未运行", "WARNING")
            return
        
        try:
            process = self.processes[process_name]
            
            # 发送 SIGTERM
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            
            # 等待最多5秒
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # 强制 SIGKILL
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait()
            
            del self.processes[process_name]
            self.update_status(process_name, False)
            self.log(f"{process_name} 已停止", "SUCCESS")
            
        except Exception as e:
            self.log(f"停止 {process_name} 时出错: {str(e)}", "ERROR")
    
    def update_status(self, process_name, is_running):
        """更新状态指示器"""
        status_map = {
            "sim": (self.lbl_sim_status, self.btn_sim, self.btn_stop_sim),
            "gridmap": (self.lbl_gridmap_status, self.btn_gridmap, self.btn_stop_gridmap),
            "bridge": (self.lbl_bridge_status, self.btn_bridge, self.btn_stop_bridge),
            "planner": (self.lbl_planner_status, self.btn_planner, self.btn_stop_planner)
        }
        
        if process_name in status_map:
            status_lbl, start_btn, stop_btn = status_map[process_name]
            
            if is_running:
                status_lbl.config(foreground="green")
                start_btn.config(state=tk.DISABLED)
                stop_btn.config(state=tk.NORMAL)
            else:
                status_lbl.config(foreground="red")
                start_btn.config(state=tk.NORMAL)
                stop_btn.config(state=tk.DISABLED)
    
    def start_all(self):
        """启动全部"""
        self.log("启动全部模块...")
        
        # 按顺序启动
        self.launch_roslaunch("sim", "ego_planner_bridge", "sim_only.launch")
        time.sleep(2)
        
        self.launch_roslaunch("gridmap", "grid_map_standalone", "test_grid_map.launch")
        time.sleep(2)
        
        self.launch_roslaunch("bridge", "ego_planner_bridge", "run_bridge.launch")
        time.sleep(2)
        
        self.launch_planner_standalone()
    
    def stop_all(self):
        """停止全部"""
        self.log("停止全部模块...")
        
        # 逆序停止
        for process_name in ["planner", "bridge", "gridmap", "sim"]:
            if process_name in self.processes:
                self.stop_process(process_name)
                time.sleep(0.5)
    
    def on_closing(self):
        """关闭窗口时的回调"""
        self.log("正在关闭控制面板...")
        self.stop_all()
        time.sleep(1)
        self.root.destroy()

def main():
    root = tk.Tk()
    app = EGOPlannerControlPanel(root)
    root.mainloop()

if __name__ == "__main__":
    main()
