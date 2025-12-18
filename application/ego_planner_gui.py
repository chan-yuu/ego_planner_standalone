#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import os
import sys
import re
from pathlib import Path
from datetime import datetime
from functools import partial

from PySide6.QtCore import (
    Qt, QTimer, QProcess, QSortFilterProxyModel, QModelIndex, Signal, QSize
)
from PySide6.QtGui import (
    QColor, QPalette, QStandardItemModel, QStandardItem, QFont, QBrush, QIcon, QPainter
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QGroupBox, QTabWidget, QLineEdit,
    QComboBox, QCheckBox, QFileDialog, QTableView, QHeaderView,
    QMessageBox, QFrame, QSizePolicy, QSpacerItem, QSplitter, QProgressBar
)

# =============================================================================
#  CONFIG & STYLES
# =============================================================================

COLORS = {
    "bg": "#0b0e14",         # 极深蓝黑
    "panel": "#151922",      # 面板背景
    "border": "#2b3245",     # 边框颜色
    "accent": "#00b0ff",     # 科技蓝（强调色）
    "text_main": "#e0e6ed",  # 主文本
    "text_dim": "#7f8c9f",   # 暗文本
    "success": "#00e676",    # 荧光绿
    "warning": "#ffea00",    # 警示黄
    "error": "#ff1744",      # 警示红
}

# 日志颜色映射
LEVEL_COLOR = {
    "INFO": QColor("#00b0ff"),
    "SUCCESS": QColor("#00e676"),
    "WARNING": QColor("#ffea00"),
    "ERROR": QColor("#ff1744"),
    "OUTPUT": QColor("#b0bec5"),
}

def now_str() -> str:
    return datetime.now().strftime("%H:%M:%S")

# =============================================================================
#  CUSTOM WIDGETS
# =============================================================================

class StatusBadge(QLabel):
    """一个带有文字状态的标签"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setFont(QFont("Segoe UI", 8, QFont.Bold))
        self.setFixedSize(60, 20)
        self.set_status(False)

    def set_status(self, active: bool):
        if active:
            self.setText("RUNNING")
            self.setStyleSheet(f"background: {COLORS['success']}; color: #000; border-radius: 2px;")
        else:
            self.setText("STOPPED")
            self.setStyleSheet(f"background: {COLORS['border']}; color: {COLORS['text_dim']}; border-radius: 2px;")

class ModuleCard(QFrame):
    """模块卡片"""
    action_requested = Signal(str, str) 

    def __init__(self, key: str, title: str, desc: str, parent=None):
        super().__init__(parent)
        self.key = key
        self.setObjectName("ModuleCard")
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(15)

        # 1. Info Area
        info_layout = QVBoxLayout()
        info_layout.setSpacing(4)
        title_lbl = QLabel(title)
        title_lbl.setObjectName("CardTitle")
        desc_lbl = QLabel(desc)
        desc_lbl.setObjectName("CardDesc")
        info_layout.addWidget(title_lbl)
        info_layout.addWidget(desc_lbl)
        layout.addLayout(info_layout, 1)

        # 2. Status
        self.badge = StatusBadge()
        layout.addWidget(self.badge)

        # 3. Actions
        self.btn_action = QPushButton("START")
        self.btn_action.setObjectName("BtnAction")
        self.btn_action.setCursor(Qt.PointingHandCursor)
        self.btn_action.setFixedWidth(70)
        self.btn_action.clicked.connect(self._on_toggle)

        self.btn_restart = QPushButton("↺")
        self.btn_restart.setObjectName("BtnIcon")
        self.btn_restart.setFixedSize(30, 30)
        self.btn_restart.setCursor(Qt.PointingHandCursor)
        self.btn_restart.setToolTip("Restart")
        self.btn_restart.setEnabled(False)
        self.btn_restart.clicked.connect(self._on_restart)

        layout.addWidget(self.btn_action)
        layout.addWidget(self.btn_restart)

    def _on_toggle(self):
        self.action_requested.emit(self.key, "toggle")

    def _on_restart(self):
        self.action_requested.emit(self.key, "restart")

    def set_running(self, running: bool):
        self.badge.set_status(running)
        self.btn_restart.setEnabled(running)
        if running:
            self.btn_action.setText("STOP")
            self.btn_action.setStyleSheet(f"border: 1px solid {COLORS['error']}; color: {COLORS['error']};")
        else:
            self.btn_action.setText("START")
            self.btn_action.setStyleSheet("")

class BuildCard(QFrame):
    """编译卡片"""
    build_requested = Signal(str)

    def __init__(self, key: str, title: str, desc: str, parent=None):
        super().__init__(parent)
        self.key = key
        self.setObjectName("ModuleCard")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(15)

        info_layout = QVBoxLayout()
        t = QLabel(title)
        t.setObjectName("CardTitle")
        d = QLabel(desc)
        d.setObjectName("CardDesc")
        info_layout.addWidget(t)
        info_layout.addWidget(d)
        layout.addLayout(info_layout, 1)

        self.lbl_status = QLabel("IDLE")
        self.lbl_status.setObjectName("BuildStatus")
        self.lbl_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_status.setFixedWidth(100)
        layout.addWidget(self.lbl_status)

        self.btn_build = QPushButton("BUILD")
        self.btn_build.setObjectName("BtnPrimary")
        self.btn_build.setCursor(Qt.PointingHandCursor)
        self.btn_build.setFixedWidth(80)
        self.btn_build.clicked.connect(lambda: self.build_requested.emit(self.key))
        layout.addWidget(self.btn_build)

    def set_building(self, building: bool):
        self.btn_build.setEnabled(not building)
        if building:
            self.lbl_status.setText("BUILDING...")
            self.lbl_status.setStyleSheet(f"color: {COLORS['warning']};")

    def set_result(self, success: bool):
        time_str = datetime.now().strftime("%H:%M")
        if success:
            self.lbl_status.setText(f"DONE {time_str}")
            self.lbl_status.setStyleSheet(f"color: {COLORS['success']}; font-weight: bold;")
        else:
            self.lbl_status.setText("FAILED")
            self.lbl_status.setStyleSheet(f"color: {COLORS['error']}; font-weight: bold;")

class LogFilterProxy(QSortFilterProxyModel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._level = "ALL"
        self._text = ""

    def set_level(self, level: str):
        self._level = level or "ALL"
        self.invalidateFilter()

    def set_text(self, text: str):
        self._text = (text or "").strip().lower()
        self.invalidateFilter()

    def filterAcceptsRow(self, source_row: int, source_parent: QModelIndex) -> bool:
        model = self.sourceModel()
        idx = model.index(source_row, 1, source_parent)
        txt_idx = model.index(source_row, 3, source_parent)
        lvl = (model.data(idx) or "").upper()
        if self._level != "ALL" and lvl != self._level: return False
        if self._text:
            msg = (model.data(txt_idx) or "").lower()
            if self._text not in msg: return False
        return True

# =============================================================================
#  MAIN WINDOW
# =============================================================================

class RuntimeManager(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Paths
        self.project_root = Path(__file__).resolve().parent.parent
        self.ego_path = self.project_root / "ego-planner"
        self.std_path = self.project_root / "planner_standalone"
        self.std_build_dir = self.std_path / "build"
        self.std_bin = self.std_build_dir / "ego_planner_standalone"

        self.processes = {}
        self.build_procs = {}
        
        self.setup_ui()
        self.setup_style()
        self._log("System Online. Workspace initialized.", "INFO", "SYS")

    def setup_style(self):
        QApplication.setStyle("Fusion")
        self.setStyleSheet(f"""
            QMainWindow {{ background: {COLORS['bg']}; }}
            QWidget {{ color: {COLORS['text_main']}; font-family: 'Segoe UI', sans-serif; }}
            
            QSplitter::handle {{ background: {COLORS['border']}; width: 1px; }}
            
            /* 顶部栏紧凑设计 */
            QFrame#TopBar {{ 
                background: {COLORS['panel']}; 
                border-bottom: 1px solid {COLORS['border']}; 
                min-height: 40px;
                max-height: 40px;
            }}
            QLabel#Logo {{ font-size: 14px; font-weight: 900; letter-spacing: 1px; color: {COLORS['accent']}; }}
            QLabel#Author {{ font-size: 12px; color: {COLORS['text_dim']}; padding-left: 10px; }}
            
            QTabWidget::pane {{ border: 0; background: {COLORS['bg']}; }}
            QTabWidget::tab-bar {{ left: 0px; }}
            QTabBar::tab {{
                background: {COLORS['bg']}; color: {COLORS['text_dim']};
                padding: 8px 16px; border-bottom: 2px solid {COLORS['border']}; font-weight: bold;
            }}
            QTabBar::tab:selected {{ color: {COLORS['accent']}; border-bottom: 2px solid {COLORS['accent']}; }}
            QTabBar::tab:hover {{ color: #fff; }}

            QFrame#ModuleCard {{
                background: {COLORS['panel']};
                border: 1px solid {COLORS['border']};
                border-left: 3px solid {COLORS['border']}; 
            }}
            QFrame#ModuleCard:hover {{ border: 1px solid #3e475e; border-left: 3px solid {COLORS['accent']}; }}
            
            QLabel#CardTitle {{ font-weight: bold; font-size: 13px; }}
            QLabel#CardDesc {{ color: {COLORS['text_dim']}; font-size: 11px; }}
            
            QPushButton {{
                background: transparent; border: 1px solid {COLORS['border']};
                color: {COLORS['text_main']}; padding: 5px 10px; font-weight: 600;
            }}
            QPushButton:hover {{ background: #1f2533; border-color: {COLORS['text_dim']}; }}
            QPushButton:pressed {{ background: #101216; }}
            
            QPushButton#BtnPrimary {{ background: {COLORS['accent']}; border: 1px solid {COLORS['accent']}; color: #000; }}
            QPushButton#BtnPrimary:hover {{ background: #40c4ff; }}
            
            QPushButton#BtnAction {{ border: 1px solid {COLORS['accent']}; color: {COLORS['accent']}; }}
            QPushButton#BtnAction:hover {{ background: rgba(0, 176, 255, 0.1); }}

            QTableView {{
                background-color: #080a0e; border: 0; gridline-color: transparent;
                font-family: 'Consolas', 'JetBrains Mono', monospace; font-size: 11px;
            }}
            QHeaderView::section {{
                background-color: #0b0e14; color: {COLORS['text_dim']};
                border: 0; border-bottom: 1px solid {COLORS['border']}; padding: 4px; font-weight: bold;
            }}
            QLineEdit, QComboBox {{
                background: {COLORS['panel']}; border: 1px solid {COLORS['border']}; color: #fff; padding: 4px;
            }}
        """)

    def setup_ui(self):
        self.setWindowTitle("EGO Planner Manager [v3.0]")
        self.resize(1280, 800)
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # === 1. TOP BAR (COMPACT) ===
        # 修正：将TopBar设置为固定高度，防止其占用过多空间
        top_bar = QFrame()
        top_bar.setObjectName("TopBar")
        top_bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed) # 关键：垂直方向固定
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(15, 0, 15, 0) # 减少上下边距
        
        logo = QLabel("RUNTIME MANAGER")
        logo.setObjectName("Logo")
        
        author = QLabel("// Author: cyun")
        author.setObjectName("Author")
        
        self.sys_status = QLabel("SYSTEM READY")
        self.sys_status.setStyleSheet(f"color: {COLORS['success']}; font-weight: bold; font-family: monospace;")
        
        top_layout.addWidget(logo)
        top_layout.addWidget(author)
        top_layout.addStretch()
        top_layout.addWidget(self.sys_status)
        
        # 添加顶部栏
        main_layout.addWidget(top_bar)

        # === 2. SPLITTER CONTENT ===
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(1)

        # --- LEFT: CONTROLS ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0) # 移除多余边距
        
        self.tabs = QTabWidget()
        
        # Tab 1: RUN
        tab_run = QWidget()
        run_layout = QVBoxLayout(tab_run)
        run_layout.setSpacing(10)
        run_layout.setContentsMargins(15, 15, 15, 15)
        
        # Global controls
        global_box = QHBoxLayout()
        btn_start_all = QPushButton("START SEQUENCE")
        btn_start_all.setObjectName("BtnPrimary")
        btn_start_all.setMinimumHeight(40)
        btn_start_all.clicked.connect(self.start_all)
        
        btn_stop_all = QPushButton("EMERGENCY STOP")
        btn_stop_all.setMinimumHeight(40)
        btn_stop_all.setStyleSheet(f"color: {COLORS['error']}; border-color: {COLORS['error']};")
        btn_stop_all.clicked.connect(self.stop_all)
        
        global_box.addWidget(btn_start_all)
        global_box.addWidget(btn_stop_all)
        run_layout.addLayout(global_box)
        
        # Spacer
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f"color: {COLORS['border']};")
        run_layout.addWidget(line)

        # Module Cards
        self.cards = {}
        modules = [
            ("sim", "SIMULATION", "Gazebo Env & Rviz"),
            ("gridmap", "GRID MAP", "Mapping Node"),
            ("bridge", "ROS BRIDGE", "Communication Layer"),
            ("planner", "PLANNER CORE", "Standalone Binary"),
        ]
        
        for k, t, d in modules:
            card = ModuleCard(k, t, d)
            card.action_requested.connect(self.handle_module_action)
            self.cards[k] = card
            run_layout.addWidget(card)

        run_layout.addStretch()
        self.tabs.addTab(tab_run, "RUNTIME CONTROL")

        # Tab 2: BUILD
        tab_build = QWidget()
        build_layout = QVBoxLayout(tab_build)
        build_layout.setSpacing(10)
        build_layout.setContentsMargins(15, 15, 15, 15)

        self.b_cards = {}
        b_modules = [
            ("ego", "EGO-PLANNER", "catkin_make (ROS)"),
            ("std", "STANDALONE", "cmake & make (C++)")
        ]
        for k, t, d in b_modules:
            bc = BuildCard(k, t, d)
            bc.build_requested.connect(self.handle_build_action)
            self.b_cards[k] = bc
            build_layout.addWidget(bc)

        build_layout.addStretch()
        self.tabs.addTab(tab_build, "BUILD SYSTEM")
        
        left_layout.addWidget(self.tabs)
        
        # --- RIGHT: LOGS ---
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)

        # Toolbar
        log_tool = QFrame()
        log_tool.setStyleSheet(f"background: {COLORS['panel']}; border-bottom: 1px solid {COLORS['border']};")
        log_tool.setFixedHeight(40) # 也可以稍微固定高度，保持对齐
        lt_layout = QHBoxLayout(log_tool)
        lt_layout.setContentsMargins(10, 0, 10, 0)
        
        lbl_term = QLabel("> TERMINAL OUTPUT")
        lbl_term.setStyleSheet(f"font-weight:bold; color: {COLORS['text_dim']}; font-family: monospace;")
        
        self.combo_lvl = QComboBox()
        self.combo_lvl.addItems(["ALL", "INFO", "WARNING", "ERROR"])
        self.combo_lvl.currentTextChanged.connect(self.update_log_filter)
        
        self.search_bar = QLineEdit()
        self.search_bar.setPlaceholderText("grep...")
        self.search_bar.textChanged.connect(self.update_log_filter)
        
        btn_cls = QPushButton("CLR")
        btn_cls.setFixedWidth(40)
        btn_cls.clicked.connect(self.clear_logs)

        lt_layout.addWidget(lbl_term)
        lt_layout.addStretch()
        lt_layout.addWidget(self.combo_lvl)
        lt_layout.addWidget(self.search_bar)
        lt_layout.addWidget(btn_cls)
        
        right_layout.addWidget(log_tool)

        # Table
        self.log_model = QStandardItemModel(0, 4)
        self.log_model.setHorizontalHeaderLabels(["TIME", "LVL", "SRC", "MESSAGE"])
        
        self.log_proxy = LogFilterProxy()
        self.log_proxy.setSourceModel(self.log_model)
        
        self.log_view = QTableView()
        self.log_view.setModel(self.log_proxy)
        self.log_view.verticalHeader().hide()
        self.log_view.setShowGrid(False)
        self.log_view.setSelectionBehavior(QTableView.SelectRows)
        
        h = self.log_view.horizontalHeader()
        h.setSectionResizeMode(0, QHeaderView.Fixed)
        h.setSectionResizeMode(1, QHeaderView.Fixed)
        h.setSectionResizeMode(2, QHeaderView.Fixed)
        h.setSectionResizeMode(3, QHeaderView.Stretch)
        self.log_view.setColumnWidth(0, 70)
        self.log_view.setColumnWidth(1, 50)
        self.log_view.setColumnWidth(2, 60)

        right_layout.addWidget(self.log_view)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 6)
        
        # 修正：将 splitter 添加到主布局，且设置拉伸因子为 1，确保占用所有剩余空间
        main_layout.addWidget(splitter, 1)

    # ================= LOGIC (保持之前的逻辑稳定) =================
    
    def handle_module_action(self, key, action):
        if action == "toggle":
            if key in self.processes:
                self.stop_process(key)
            else:
                self.start_process(key)
        elif action == "restart":
            self.stop_process(key)
            QTimer.singleShot(1000, lambda: self.start_process(key))

    def handle_build_action(self, key):
        if key in self.build_procs: return
        
        card = self.b_cards[key]
        card.set_building(True)
        self._log(f"Build sequence started: {key}", "INFO", "BLD")
        
        cmd = ""
        if key == "ego":
            cmd = f"source /opt/ros/noetic/setup.bash && cd {self.ego_path} && catkin_make"
        elif key == "std":
            self.std_build_dir.mkdir(parents=True, exist_ok=True)
            cmd = f"cd {self.std_build_dir} && cmake .. && make -j$(nproc)"
            
        self.run_process(key, cmd, is_build=True)

    def start_process(self, key):
        if key in self.processes: return
        
        cmds = {
            "sim": f"source {self.ego_path}/devel/setup.bash && roslaunch ego_planner_bridge sim_only.launch",
            "gridmap": f"source {self.ego_path}/devel/setup.bash && roslaunch grid_map_standalone test_grid_map.launch",
            "bridge": f"source {self.ego_path}/devel/setup.bash && roslaunch ego_planner_bridge run_bridge.launch",
            "planner": f"{self.std_bin}"
        }
        
        if key == "planner" and not self.std_bin.exists():
            self._log("Binary not found. Please build first.", "ERROR", "SYS")
            return

        wd = str(self.std_build_dir) if key == "planner" else str(self.project_root)
        self.run_process(key, cmds[key], cwd=wd)

    def run_process(self, key, cmd, cwd=None, is_build=False):
        full_cmd = f"bash -lc '{cmd}'"
        
        p = QProcess(self)
        if cwd: p.setWorkingDirectory(cwd)
        p.setProcessChannelMode(QProcess.MergedChannels)
        
        # 使用 partial 解决闭包问题
        p.readyReadStandardOutput.connect(partial(self.read_output, p, key))
        p.finished.connect(partial(self.on_finished, key, is_build=is_build))
        
        if is_build:
            self.build_procs[key] = p
        else:
            self.processes[key] = p
            self.cards[key].set_running(True)
            self._log(f"Process spawned. PID: {p.processId()}", "SUCCESS", key.upper())

        p.start("bash", ["-c", full_cmd])

    def stop_process(self, key):
        if key in self.processes:
            self._log("Sending SIGTERM...", "WARNING", key.upper())
            self.processes[key].terminate()
            QTimer.singleShot(2000, lambda: self.force_kill(key))

    def force_kill(self, key):
        if key in self.processes and self.processes[key].state() != QProcess.NotRunning:
            self.processes[key].kill()

    def on_finished(self, key, code, status, is_build=False):
        # 注意：QProcess.finished 信号发送 (code, status)，所以这里需要接收 status
        if is_build:
            self.b_cards[key].set_building(False)
            self.b_cards[key].set_result(code == 0)
            if key in self.build_procs: del self.build_procs[key]
            msg = "Build Complete" if code == 0 else f"Build Failed (Code {code})"
            lvl = "SUCCESS" if code == 0 else "ERROR"
            self._log(msg, lvl, "BLD")
        else:
            self.cards[key].set_running(False)
            if key in self.processes: del self.processes[key]
            self._log(f"Process exited (Code {code})", "WARNING", key.upper())

    def read_output(self, proc, key):
        try:
            data = proc.readAllStandardOutput().data().decode('utf-8', errors='ignore')
            for line in data.splitlines():
                if not line.strip(): continue
                lvl = "OUTPUT"
                clean_line = self.strip_ansi(line)
                low = clean_line.lower()
                if "error" in low or "fail" in low: lvl = "ERROR"
                elif "warn" in low: lvl = "WARNING"
                self._log(clean_line, lvl, key.upper())
        except Exception:
            pass

    def strip_ansi(self, text):
        return re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])').sub('', text)

    def _log(self, msg, lvl, src):
        row = [
            QStandardItem(now_str()),
            QStandardItem(lvl),
            QStandardItem(src),
            QStandardItem(msg)
        ]
        color = LEVEL_COLOR.get(lvl, QColor("#fff"))
        for i in range(4):
            row[i].setForeground(QBrush(color if lvl in ["ERROR", "SUCCESS", "INFO"] else QColor("#e0e6ed")))
            if i == 0: row[i].setForeground(QBrush(QColor(COLORS['text_dim'])))
            if i == 1: row[i].setFont(QFont("Consolas", 9, QFont.Bold))

        self.log_model.appendRow(row)
        if self.log_view.verticalScrollBar().value() > self.log_view.verticalScrollBar().maximum() - 50:
            self.log_view.scrollToBottom()

    def update_log_filter(self):
        self.log_proxy.set_level(self.combo_lvl.currentText())
        self.log_proxy.set_text(self.search_bar.text())

    def clear_logs(self):
        self.log_model.removeRows(0, self.log_model.rowCount())

    def start_all(self):
        keys = ["sim", "gridmap", "bridge", "planner"]
        for i, k in enumerate(keys):
            QTimer.singleShot(i*1500, partial(self.start_process, k))

    def stop_all(self):
        for k in list(self.processes.keys()):
            self.stop_process(k)
            
    def closeEvent(self, event):
        self.stop_all()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = RuntimeManager()
    win.show()
    sys.exit(app.exec())