#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于PyQt5的现代化GUI界面
"""

import os
import sys
import subprocess
import signal
from pathlib import Path
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QTextEdit, 
                             QGroupBox, QGridLayout, QFrame, QSplitter)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QFont, QColor, QPalette, QTextCursor, QPainter, QPen

class ProcessThread(QThread):
    """进程执行线程"""
    log_signal = pyqtSignal(str, str)  # message, level
    status_signal = pyqtSignal(str, bool)  # process_name, is_running
    
    def __init__(self, process_name, command, cwd=None, need_source=False):
        super().__init__()
        self.process_name = process_name
        self.command = command
        self.cwd = cwd
        self.need_source = need_source
        self.process = None
        self._is_running = False
    
    def run(self):
        try:
            self._is_running = True
            self.log_signal.emit(f"启动 {self.process_name}...", "INFO")
            
            self.process = subprocess.Popen(
                self.command,
                shell=True,
                cwd=self.cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                executable='/bin/bash',  # 使用bash执行source命令
                preexec_fn=os.setsid,
                bufsize=1,
                universal_newlines=True
            )
            
            self.status_signal.emit(self.process_name, True)
            self.log_signal.emit(f"{self.process_name} 启动成功", "SUCCESS")
            
            # 读取输出
            for line in iter(self.process.stdout.readline, ''):
                if line:
                    self.log_signal.emit(line.strip(), "OUTPUT")
            
            self.process.wait()
            
        except Exception as e:
            self.log_signal.emit(f"{self.process_name} 启动失败: {str(e)}", "ERROR")
        finally:
            self._is_running = False
            self.status_signal.emit(self.process_name, False)
            self.log_signal.emit(f"{self.process_name} 已停止", "INFO")
    
    def stop(self):
        if self.process and self._is_running:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except:
                pass

class CompileThread(QThread):
    """编译线程"""
    log_signal = pyqtSignal(str, str)
    finished_signal = pyqtSignal(bool)
    
    def __init__(self, command, project_name):
        super().__init__()
        self.command = command
        self.project_name = project_name
    
    def run(self):
        try:
            self.log_signal.emit(f"开始编译 {self.project_name}...", "INFO")
            
            result = subprocess.run(
                self.command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=300,
                executable='/bin/bash'  # 使用bash执行source命令
            )
            
            if result.stdout:
                for line in result.stdout.split('\n'):
                    if line.strip():
                        self.log_signal.emit(line, "OUTPUT")
            
            if result.stderr:
                for line in result.stderr.split('\n'):
                    if line.strip():
                        self.log_signal.emit(line, "ERROR")
            
            if result.returncode == 0:
                self.log_signal.emit(f"{self.project_name} 编译成功", "SUCCESS")
                self.finished_signal.emit(True)
            else:
                self.log_signal.emit(f"{self.project_name} 编译失败 (返回码: {result.returncode})", "ERROR")
                self.finished_signal.emit(False)
                
        except Exception as e:
            self.log_signal.emit(f"编译出错: {str(e)}", "ERROR")
            self.finished_signal.emit(False)

class StatusIndicator(QLabel):
    """状态指示灯"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(20, 20)
        self.is_active = False
        self.update_style()
    
    def set_active(self, active):
        self.is_active = active
        self.update_style()
    
    def update_style(self):
        color = "#66bb6a" if self.is_active else "#ef5350"
        glow_color = "#81c784" if self.is_active else "#e57373"
        self.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                border-radius: 10px;
                border: 2px solid {glow_color};
            }}
        """)

class ModuleButton(QPushButton):
    """模块控制按钮"""
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setMinimumHeight(45)
        self.setFont(QFont("Arial", 11))
        self.setCursor(Qt.PointingHandCursor)
        self.update_style(False)
    
    def update_style(self, is_running):
        if is_running:
            self.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #1a237e, stop:1 #283593);
                    color: #e8eaf6;
                    border: 2px solid #5c6bc0;
                    border-radius: 8px;
                    padding: 8px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #283593, stop:1 #3949ab);
                    border: 2px solid #7986cb;
                }
                QPushButton:disabled {
                    background: #263238;
                    color: #546e7a;
                    border: 2px solid #37474f;
                }
            """)
        else:
            self.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #1b1b2f, stop:1 #162447);
                    color: #9fa8da;
                    border: 2px solid #5c6bc0;
                    border-radius: 8px;
                    padding: 8px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #162447, stop:1 #1f4068);
                    border: 2px solid #7986cb;
                }
                QPushButton:pressed {
                    background: #0d1b2a;
                }
                QPushButton:disabled {
                    background: #1a1a1a;
                    color: #424242;
                    border: 2px solid #2c2c2c;
                }
            """)

class EGOPlannerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 项目路径
        # GUI在 application/ 目录中,项目根在上级目录
        self.project_root = Path(__file__).resolve().parent.parent
        self.ego_planner_path = self.project_root / "ego-planner"
        self.planner_standalone_path = self.project_root / "planner_standalone"
        
        # 进程管理
        self.processes = {}
        self.threads = {}
        
        # 初始化UI
        self.init_ui()
        
        # 定时器用于闪烁效果
        self.blink_timer = QTimer()
        self.blink_timer.timeout.connect(self.update_blink)
        self.blink_state = False
    
    def init_ui(self):
        self.setWindowTitle("Runtime Manager")
        self.setGeometry(100, 100, 1400, 900)
        
        # 设置暗色科技风格
        self.set_dark_theme()
        
        # 中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # 标题
        title_label = QLabel("EGO-PLANNER 控制中心")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Arial", 24, QFont.Bold))
        title_label.setStyleSheet("""
            QLabel {
                color: #9fa8da;
                padding: 20px;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 rgba(92, 107, 192, 0.15), 
                    stop:0.5 rgba(121, 134, 203, 0.25),
                    stop:1 rgba(92, 107, 192, 0.15));
                border: 2px solid #5c6bc0;
                border-radius: 15px;
            }
        """)
        main_layout.addWidget(title_label)
        
        # 分割器
        splitter = QSplitter(Qt.Horizontal)
        
        # 左侧控制面板
        left_panel = self.create_control_panel()
        splitter.addWidget(left_panel)
        
        # 右侧日志面板
        right_panel = self.create_log_panel()
        splitter.addWidget(right_panel)
        
        splitter.setSizes([700, 700])
        main_layout.addWidget(splitter)
        
        # 底部作者信息
        author_label = QLabel("© 2025 Contributed by CYUN")
        author_label.setAlignment(Qt.AlignCenter)
        author_label.setFont(QFont("Arial", 15))
        author_label.setStyleSheet("""
            QLabel {
                color: #6c757d;
                padding: 10px;
                background: transparent;
                border-top: 1px solid #30363d;
            }
        """)
        main_layout.addWidget(author_label)
        
        # 状态栏
        self.statusBar().setStyleSheet("""
            QStatusBar {
                background: #0d1117;
                color: #9fa8da;
                border-top: 1px solid #30363d;
            }
        """)
        self.statusBar().showMessage("系统就绪")
    
    def set_dark_theme(self):
        """设置暗色主题"""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(13, 17, 23))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(22, 27, 34))
        palette.setColor(QPalette.AlternateBase, QColor(32, 38, 46))
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(33, 38, 45))
        palette.setColor(QPalette.ButtonText, Qt.white)
        self.setPalette(palette)
        
        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #0d1117, stop:1 #161b22);
            }
            QGroupBox {
                color: #9fa8da;
                border: 2px solid #5c6bc0;
                border-radius: 10px;
                margin-top: 15px;
                padding-top: 15px;
                font-weight: bold;
                font-size: 14px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 20px;
                padding: 0 5px;
            }
        """)
    
    def create_control_panel(self):
        """创建控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setSpacing(15)
        
        # 编译区域
        compile_group = QGroupBox("📦 编译模块")
        compile_layout = QGridLayout()
        compile_layout.setSpacing(10)
        
        # EGO-Planner编译
        self.btn_compile_ego = ModuleButton("编译 EGO-Planner")
        self.btn_compile_ego.clicked.connect(self.compile_ego_planner)
        compile_layout.addWidget(self.btn_compile_ego, 0, 0)
        
        self.lbl_ego_compile_status = QLabel("未编译")
        self.lbl_ego_compile_status.setStyleSheet("color: #6c757d; font-size: 12px;")
        compile_layout.addWidget(self.lbl_ego_compile_status, 0, 1)
        
        # 独立规划器编译
        self.btn_compile_standalone = ModuleButton("编译 独立规划器")
        self.btn_compile_standalone.clicked.connect(self.compile_planner_standalone)
        compile_layout.addWidget(self.btn_compile_standalone, 1, 0)
        
        self.lbl_standalone_compile_status = QLabel("未编译")
        self.lbl_standalone_compile_status.setStyleSheet("color: #6c757d; font-size: 12px;")
        compile_layout.addWidget(self.lbl_standalone_compile_status, 1, 1)
        
        compile_group.setLayout(compile_layout)
        layout.addWidget(compile_group)
        
        # 运行模块区域
        run_group = QGroupBox("🚀 运行模块")
        run_layout = QGridLayout()
        run_layout.setSpacing(10)
        
        # 仿真环境
        self.btn_sim = ModuleButton("启动仿真环境")
        self.btn_sim.clicked.connect(lambda: self.toggle_module("sim"))
        run_layout.addWidget(self.btn_sim, 0, 0)
        
        self.ind_sim = StatusIndicator()
        run_layout.addWidget(self.ind_sim, 0, 1)
        
        # Grid Map
        self.btn_gridmap = ModuleButton("启动 Grid Map")
        self.btn_gridmap.clicked.connect(lambda: self.toggle_module("gridmap"))
        run_layout.addWidget(self.btn_gridmap, 1, 0)
        
        self.ind_gridmap = StatusIndicator()
        run_layout.addWidget(self.ind_gridmap, 1, 1)
        
        # ROS Bridge
        self.btn_bridge = ModuleButton("启动 ROS Bridge")
        self.btn_bridge.clicked.connect(lambda: self.toggle_module("bridge"))
        run_layout.addWidget(self.btn_bridge, 2, 0)
        
        self.ind_bridge = StatusIndicator()
        run_layout.addWidget(self.ind_bridge, 2, 1)
        
        # 独立规划器
        self.btn_planner = ModuleButton("启动独立规划器")
        self.btn_planner.clicked.connect(lambda: self.toggle_module("planner"))
        run_layout.addWidget(self.btn_planner, 3, 0)
        
        self.ind_planner = StatusIndicator()
        run_layout.addWidget(self.ind_planner, 3, 1)
        
        run_group.setLayout(run_layout)
        layout.addWidget(run_group)
        
        # 快捷操作
        quick_group = QGroupBox("⚡ 快捷操作")
        quick_layout = QHBoxLayout()
        
        self.btn_start_all = QPushButton("🚀 启动全部")
        self.btn_start_all.setMinimumHeight(50)
        self.btn_start_all.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2e7d32, stop:1 #43a047);
                color: white;
                border: none;
                border-radius: 10px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #388e3c, stop:1 #4caf50);
            }
        """)
        self.btn_start_all.clicked.connect(self.start_all)
        quick_layout.addWidget(self.btn_start_all)
        
        self.btn_stop_all = QPushButton("🛑 停止全部")
        self.btn_stop_all.setMinimumHeight(50)
        self.btn_stop_all.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #c62828, stop:1 #e53935);
                color: white;
                border: none;
                border-radius: 10px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #d32f2f, stop:1 #f44336);
            }
        """)
        self.btn_stop_all.clicked.connect(self.stop_all)
        quick_layout.addWidget(self.btn_stop_all)
        
        quick_group.setLayout(quick_layout)
        layout.addWidget(quick_group)
        
        layout.addStretch()
        
        return panel
    
    def create_log_panel(self):
        """创建日志面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 日志标题
        log_title = QLabel("📋 系统日志")
        log_title.setStyleSheet("""
            QLabel {
                color: #9fa8da;
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
            }
        """)
        layout.addWidget(log_title)
        
        # 日志文本框
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 10))
        self.log_text.setStyleSheet("""
            QTextEdit {
                background: #0d1117;
                color: #c9d1d9;
                border: 2px solid #5c6bc0;
                border-radius: 10px;
                padding: 10px;
            }
        """)
        layout.addWidget(self.log_text)
        
        # 清空按钮
        btn_clear = QPushButton("🗑️ 清空日志")
        btn_clear.setMinimumHeight(40)
        btn_clear.setStyleSheet("""
            QPushButton {
                background: #21262d;
                color: #9fa8da;
                border: 1px solid #5c6bc0;
                border-radius: 8px;
                font-size: 12px;
            }
            QPushButton:hover {
                background: #30363d;
                border: 1px solid #7986cb;
            }
        """)
        btn_clear.clicked.connect(self.clear_log)
        layout.addWidget(btn_clear)
        
        return panel
    
    def log(self, message, level="INFO"):
        """输出日志"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        color_map = {
            "INFO": "#7986cb",
            "SUCCESS": "#66bb6a",
            "ERROR": "#ef5350",
            "WARNING": "#ffa726",
            "OUTPUT": "#90a4ae"
        }
        
        color = color_map.get(level, "#c9d1d9")
        
        if level == "OUTPUT":
            formatted = f'<span style="color: {color};">{message}</span>'
        else:
            formatted = f'<span style="color: #6c757d;">[{timestamp}]</span> ' \
                       f'<span style="color: {color};">[{level}]</span> ' \
                       f'<span style="color: {color};">{message}</span>'
        
        self.log_text.append(formatted)
        self.log_text.moveCursor(QTextCursor.End)
        
        # 更新状态栏
        self.statusBar().showMessage(message)
    
    def clear_log(self):
        """清空日志"""
        self.log_text.clear()
        self.log("日志已清空", "INFO")
    
    def compile_ego_planner(self):
        """编译 EGO-Planner"""
        self.btn_compile_ego.setEnabled(False)
        self.lbl_ego_compile_status.setText("编译中...")
        self.lbl_ego_compile_status.setStyleSheet("color: #ffaa00;")
        
        cmd = f"bash -c 'cd {self.ego_planner_path} && " \
              f"source /opt/ros/noetic/setup.bash && " \
              f"catkin_make'"
        
        thread = CompileThread(cmd, "EGO-Planner")
        thread.log_signal.connect(self.log)
        thread.finished_signal.connect(lambda success: self.on_compile_finished(
            "ego", success, self.lbl_ego_compile_status, self.btn_compile_ego))
        thread.start()
        self.threads["compile_ego"] = thread
    
    def compile_planner_standalone(self):
        """编译独立规划器"""
        self.btn_compile_standalone.setEnabled(False)
        self.lbl_standalone_compile_status.setText("编译中...")
        self.lbl_standalone_compile_status.setStyleSheet("color: #ffaa00;")
        
        build_dir = self.planner_standalone_path / "build"
        if not build_dir.exists():
            build_dir.mkdir(parents=True)
        
        cmd = f"bash -c 'cd {build_dir} && cmake .. && make -j4'"
        
        thread = CompileThread(cmd, "独立规划器")
        thread.log_signal.connect(self.log)
        thread.finished_signal.connect(lambda success: self.on_compile_finished(
            "standalone", success, self.lbl_standalone_compile_status, self.btn_compile_standalone))
        thread.start()
        self.threads["compile_standalone"] = thread
    
    def on_compile_finished(self, name, success, status_label, button):
        """编译完成回调"""
        if success:
            status_label.setText("✅ 编译成功")
            status_label.setStyleSheet("color: #66bb6a; font-weight: bold;")
        else:
            status_label.setText("❌ 编译失败")
            status_label.setStyleSheet("color: #ef5350; font-weight: bold;")
        
        button.setEnabled(True)
    
    def toggle_module(self, module_name):
        """切换模块状态"""
        if module_name in self.processes:
            self.stop_module(module_name)
        else:
            self.start_module(module_name)
    
    def start_module(self, module_name):
        """启动模块"""
        if module_name in self.processes:
            self.log(f"{module_name} 已经在运行", "WARNING")
            return
        
        module_config = {
            "sim": {
                "cmd": f"source {self.ego_planner_path}/devel/setup.bash && "
                       f"roslaunch ego_planner_bridge sim_only.launch",
                "need_source": True
            },
            "gridmap": {
                "cmd": f"source {self.ego_planner_path}/devel/setup.bash && "
                       f"roslaunch grid_map_standalone test_grid_map.launch",
                "need_source": True
            },
            "bridge": {
                "cmd": f"source {self.ego_planner_path}/devel/setup.bash && "
                       f"roslaunch ego_planner_bridge run_bridge.launch",
                "need_source": True
            },
            "planner": {
                "cmd": str(self.planner_standalone_path / "build" / "ego_planner_standalone"),
                "cwd": str(self.planner_standalone_path / "build"),
                "need_source": False
            }
        }
        
        if module_name not in module_config:
            return
        
        config = module_config[module_name]
        thread = ProcessThread(
            module_name, 
            config["cmd"], 
            config.get("cwd"),
            config["need_source"]
        )
        thread.log_signal.connect(self.log)
        thread.status_signal.connect(self.update_module_status)
        thread.start()
        
        self.processes[module_name] = thread
    
    def stop_module(self, module_name):
        """停止模块"""
        if module_name in self.processes:
            thread = self.processes[module_name]
            thread.stop()
            thread.wait()
            del self.processes[module_name]
            self.update_module_status(module_name, False)
            self.log(f"{module_name} 已停止", "INFO")
    
    def update_module_status(self, module_name, is_running):
        """更新模块状态"""
        status_map = {
            "sim": (self.btn_sim, self.ind_sim),
            "gridmap": (self.btn_gridmap, self.ind_gridmap),
            "bridge": (self.btn_bridge, self.ind_bridge),
            "planner": (self.btn_planner, self.ind_planner)
        }
        
        if module_name in status_map:
            button, indicator = status_map[module_name]
            indicator.set_active(is_running)
            button.update_style(is_running)
            
            if is_running:
                button.setText(f"停止 {button.text().split('启动')[-1].strip()}")
            else:
                button.setText(f"启动{button.text().split('停止')[-1].strip()}")
    
    def start_all(self):
        """启动全部"""
        self.log("启动全部模块...", "INFO")
        
        modules = ["sim", "gridmap", "bridge", "planner"]
        for i, module in enumerate(modules):
            QTimer.singleShot(i * 2000, lambda m=module: self.start_module(m))
    
    def stop_all(self):
        """停止全部"""
        self.log("停止全部模块...", "INFO")
        
        for module_name in list(self.processes.keys()):
            self.stop_module(module_name)
    
    def update_blink(self):
        """更新闪烁状态"""
        self.blink_state = not self.blink_state
    
    def closeEvent(self, event):
        """关闭事件"""
        self.log("正在关闭控制面板...", "INFO")
        self.stop_all()
        event.accept()

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    window = EGOPlannerGUI()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
