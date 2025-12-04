#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import json
import os
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit, QMessageBox, QFileDialog, QGroupBox,
    QCheckBox, QScrollArea, QFrame, QSplitter, QButtonGroup, QRadioButton
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QPalette, QColor
import rclpy
from states import RobotJointStates


class RecordItemWidget(QFrame):
    """Widget for displaying a single recorded element"""
    
    def __init__(self, file_index, record_data, parent_panel):
        super().__init__()
        self.file_index = file_index
        self.key_index = None
        self.record_data = record_data
        self.parent_panel = parent_panel
        
        self.init_ui()
        self.update_display()
    
    def init_ui(self):
        self.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.setLineWidth(1)
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 6, 8, 6)
        main_layout.setSpacing(4)
        
        # Header row: index, type, description, key_point toggle
        header_layout = QHBoxLayout()
        
        # Index label
        self.index_label = QLabel("")
        self.index_label.setFont(QFont("Arial", 9, QFont.Bold))
        self.index_label.setFixedWidth(35)
        header_layout.addWidget(self.index_label)
        
        # Type label (key_action)
        key_action = self.record_data.get('key_action', 'unknown')
        self.type_label = QLabel(key_action.upper())
        self.type_label.setFont(QFont("Arial", 10, QFont.Bold))
        header_layout.addWidget(self.type_label)
        
        # Description
        desc = self.record_data.get('description', '')
        self.desc_label = QLabel(f"- {desc}" if desc else "")
        self.desc_label.setFont(QFont("Arial", 9))
        header_layout.addWidget(self.desc_label, 1)
        
        # Key point toggle buttons
        self.key_btn_group = QButtonGroup(self)
        self.key_true_btn = QRadioButton("KEY")
        self.key_false_btn = QRadioButton("NORMAL")
        self.key_true_btn.setFont(QFont("Arial", 8))
        self.key_false_btn.setFont(QFont("Arial", 8))
        self.key_btn_group.addButton(self.key_true_btn, 1)
        self.key_btn_group.addButton(self.key_false_btn, 0)
        
        if self.record_data.get('key_point', False):
            self.key_true_btn.setChecked(True)
        else:
            self.key_false_btn.setChecked(True)
        
        self.key_btn_group.buttonClicked.connect(self.on_key_point_changed)
        
        key_layout = QHBoxLayout()
        key_layout.setSpacing(2)
        key_layout.addWidget(self.key_true_btn)
        key_layout.addWidget(self.key_false_btn)
        header_layout.addLayout(key_layout)
        
        main_layout.addLayout(header_layout)
        
        # Value display area (collapsible)
        self.value_frame = QFrame()
        value_layout = QVBoxLayout(self.value_frame)
        value_layout.setContentsMargins(40, 2, 4, 2)
        value_layout.setSpacing(1)
        
        self.value_label = QLabel()
        self.value_label.setFont(QFont("Consolas", 8))
        self.value_label.setStyleSheet("color: #333;")
        self.value_label.setWordWrap(True)
        value_layout.addWidget(self.value_label)
        
        main_layout.addWidget(self.value_frame)
        
        self.update_value_display()
    
    def update_value_display(self):
        """Update the value display based on key_action"""
        key_action = self.record_data.get('key_action', '')
        
        if key_action == 'left_arm':
            values = self.record_data.get('left_arm', [])
            self.value_label.setText(f"[{', '.join(f'{v:.4f}' for v in values)}]")
        elif key_action == 'right_arm':
            values = self.record_data.get('right_arm', [])
            self.value_label.setText(f"[{', '.join(f'{v:.4f}' for v in values)}]")
        elif key_action == 'left_gripper':
            values = self.record_data.get('left_gripper', [0])
            self.value_label.setText(f"Position: {values[0]:.4f}")
        elif key_action == 'right_gripper':
            values = self.record_data.get('right_gripper', [0])
            self.value_label.setText(f"Position: {values[0]:.4f}")
        elif key_action == 'pause':
            value = self.record_data.get('pause', 0)
            self.value_label.setText(f"Duration: {value}s")
        else:
            self.value_label.setText("Unknown action")
    
    def set_key_index(self, key_index):
        """Set the index among key_point=T records"""
        self.key_index = key_index
        self.index_label.setText(f"#{key_index + 1}" if key_index is not None else "")
    
    def update_display(self):
        """Update visual style based on key_point status"""
        is_key = self.record_data.get('key_point', False)
        if is_key:
            self.setStyleSheet("RecordItemWidget { background-color: #fff; border: 1px solid #4CAF50; border-radius: 4px; }")
            self.type_label.setStyleSheet("color: #2196F3; font-weight: bold;")
            self.desc_label.setStyleSheet("color: #666;")
            self.value_label.setStyleSheet("color: #333;")
        else:
            self.setStyleSheet("RecordItemWidget { background-color: #e8e8e8; border: 1px solid #bbb; border-radius: 4px; }")
            self.type_label.setStyleSheet("color: #999; font-weight: bold;")
            self.desc_label.setStyleSheet("color: #aaa;")
            self.value_label.setStyleSheet("color: #999;")
    
    def on_key_point_changed(self, button):
        """Handle key_point toggle change"""
        new_value = (button == self.key_true_btn)
        self.record_data['key_point'] = new_value
        self.update_display()
        self.parent_panel.update_record_in_file(self.file_index, self.record_data)
        self.parent_panel.update_key_indices()


class JointRecorderPanel(QMainWindow):
    def __init__(self, json_file='/ros2_ws/src/process/data/raw_data.json'):
        super().__init__()
        self.json_file = json_file
        self.record_widgets = []
        self.show_normal_items = True
        
        # Initialize ROS2 components
        if not rclpy.ok():
            rclpy.init()
            
        try:
            self.robot_states = RobotJointStates(timeout_sec=5.0)
        except Exception as e:
            QMessageBox.warning(self, "Warning", f"Failed to initialize RobotJointStates: {str(e)}\nUsing default values.")
            self.robot_states = RobotJointStates(timeout_sec=0.1)
        
        # Current joint states
        self.current_states = {}
        
        # Initialize UI
        self.init_ui()
        
        # Initialize JSON file
        self.init_json_file()
        
        # Timer for updating joint display
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_joint_display)
        self.timer.start(100)
        
    def init_ui(self):
        self.setWindowTitle('Robot Joint State Recorder')
        self.setGeometry(100, 100, 1400, 800)
        
        # Main widget with splitter
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Create splitter for left and right panels
        splitter = QSplitter(Qt.Horizontal)
        
        # === LEFT PANEL ===
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # File Settings
        file_group = QGroupBox("File Settings")
        file_layout = QHBoxLayout()
        
        self.file_path_label = QLabel("File Path:")
        self.file_path_edit = QLineEdit(self.json_file)
        self.browse_btn = QPushButton("Browse")
        self.browse_btn.clicked.connect(self.browse_file)
        
        file_layout.addWidget(self.file_path_label)
        file_layout.addWidget(self.file_path_edit)
        file_layout.addWidget(self.browse_btn)
        file_group.setLayout(file_layout)
        
        # Current Joint States
        status_group = QGroupBox("Current Joint States")
        status_layout = QGridLayout()
        
        # Left Arm
        left_arm_group = QGroupBox("Left Arm")
        left_arm_layout = QVBoxLayout()
        self.left_arm_labels = {}
        for i in range(1, 7):
            label = QLabel(f"joint{i}: 0.0000")
            label.setFont(QFont("Consolas", 9))
            self.left_arm_labels[f'left_arm_joint{i}'] = label
            left_arm_layout.addWidget(label)
        left_arm_group.setLayout(left_arm_layout)
        
        # Right Arm
        right_arm_group = QGroupBox("Right Arm")
        right_arm_layout = QVBoxLayout()
        self.right_arm_labels = {}
        for i in range(1, 7):
            label = QLabel(f"joint{i}: 0.0000")
            label.setFont(QFont("Consolas", 9))
            self.right_arm_labels[f'right_arm_joint{i}'] = label
            right_arm_layout.addWidget(label)
        right_arm_group.setLayout(right_arm_layout)
        
        # Grippers
        gripper_group = QGroupBox("Grippers")
        gripper_layout = QVBoxLayout()
        self.left_gripper_label = QLabel("Left: 0.0000")
        self.right_gripper_label = QLabel("Right: 0.0000")
        self.left_gripper_label.setFont(QFont("Consolas", 9))
        self.right_gripper_label.setFont(QFont("Consolas", 9))
        gripper_layout.addWidget(self.left_gripper_label)
        gripper_layout.addWidget(self.right_gripper_label)
        gripper_group.setLayout(gripper_layout)
        
        # Other Joints
        other_group = QGroupBox("Other")
        other_layout = QVBoxLayout()
        self.head_yaw_label = QLabel("Head Yaw: 0.0000")
        self.head_pitch_label = QLabel("Head Pitch: 0.0000")
        self.slide_label = QLabel("Slide: 0.0000")
        self.head_yaw_label.setFont(QFont("Consolas", 9))
        self.head_pitch_label.setFont(QFont("Consolas", 9))
        self.slide_label.setFont(QFont("Consolas", 9))
        other_layout.addWidget(self.head_yaw_label)
        other_layout.addWidget(self.head_pitch_label)
        other_layout.addWidget(self.slide_label)
        other_group.setLayout(other_layout)
        
        status_layout.addWidget(left_arm_group, 0, 0)
        status_layout.addWidget(right_arm_group, 0, 1)
        status_layout.addWidget(gripper_group, 1, 0)
        status_layout.addWidget(other_group, 1, 1)
        status_group.setLayout(status_layout)
        
        # Record Actions
        button_group = QGroupBox("Record Actions (Click to Save)")
        button_layout = QHBoxLayout()
        
        self.left_gripper_btn = QPushButton('Left Gripper')
        self.right_gripper_btn = QPushButton('Right Gripper')
        self.left_arm_btn = QPushButton('Left Arm')
        self.right_arm_btn = QPushButton('Right Arm')
        self.pause_btn = QPushButton('Pause')
        
        # Style action buttons
        btn_style = "QPushButton { padding: 8px; font-weight: bold; }"
        for btn in [self.left_gripper_btn, self.right_gripper_btn, 
                    self.left_arm_btn, self.right_arm_btn, self.pause_btn]:
            btn.setStyleSheet(btn_style)
        
        self.left_gripper_btn.clicked.connect(lambda: self.record_raw_data('left_gripper'))
        self.right_gripper_btn.clicked.connect(lambda: self.record_raw_data('right_gripper'))
        self.left_arm_btn.clicked.connect(lambda: self.record_raw_data('left_arm'))
        self.right_arm_btn.clicked.connect(lambda: self.record_raw_data('right_arm'))
        self.pause_btn.clicked.connect(lambda: self.record_raw_data('pause'))
        
        button_layout.addWidget(self.left_gripper_btn)
        button_layout.addWidget(self.right_gripper_btn)
        button_layout.addWidget(self.left_arm_btn)
        button_layout.addWidget(self.right_arm_btn)
        button_layout.addWidget(self.pause_btn)
        button_group.setLayout(button_layout)
        
        # Record Settings
        settings_group = QGroupBox("Record Settings")
        settings_layout = QGridLayout()
        
        self.key_point_checkbox = QCheckBox('Key Point (checked=True)')
        self.key_point_checkbox.setChecked(True)
        settings_layout.addWidget(self.key_point_checkbox, 0, 0, 1, 2)
        
        settings_layout.addWidget(QLabel('Pause Time (s):'), 1, 0)
        self.pause_input = QLineEdit('2')
        settings_layout.addWidget(self.pause_input, 1, 1)
        
        settings_layout.addWidget(QLabel('Description:'), 2, 0)
        self.desc_input = QLineEdit('')
        settings_layout.addWidget(self.desc_input, 2, 1)
        
        settings_group.setLayout(settings_layout)
        
        # Operation Log
        log_group = QGroupBox("Operation Log")
        log_layout = QVBoxLayout()
        self.log_text_area = QTextEdit()
        self.log_text_area.setReadOnly(True)
        self.log_text_area.setMaximumHeight(150)
        log_layout.addWidget(self.log_text_area)
        log_group.setLayout(log_layout)
        
        # Control Buttons
        control_layout = QHBoxLayout()
        new_file_btn = QPushButton('New File')
        status_btn = QPushButton('File Status')
        load_btn = QPushButton('Refresh')
        clear_data_btn = QPushButton('Clear Data')
        clear_log_btn = QPushButton('Clear Log')
        
        # Style clear buttons with red color
        clear_btn_style = """
            QPushButton {
                background-color: #f44336;
                color: white;
                font-weight: bold;
                padding: 5px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #d32f2f;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
        """
        clear_data_btn.setStyleSheet(clear_btn_style)
        clear_log_btn.setStyleSheet(clear_btn_style)
        
        new_file_btn.clicked.connect(self.create_new_file)
        status_btn.clicked.connect(self.show_file_status)
        load_btn.clicked.connect(self.load_json)
        clear_data_btn.clicked.connect(self.clear_data)
        clear_log_btn.clicked.connect(self.clear_log)
        
        control_layout.addWidget(new_file_btn)
        control_layout.addWidget(status_btn)
        control_layout.addWidget(load_btn)
        control_layout.addWidget(clear_data_btn)
        control_layout.addWidget(clear_log_btn)
        
        # Add all to left layout
        left_layout.addWidget(file_group)
        left_layout.addWidget(status_group)
        left_layout.addWidget(button_group)
        left_layout.addWidget(settings_group)
        left_layout.addWidget(log_group)
        left_layout.addLayout(control_layout)
        
        # === RIGHT PANEL - Record Visualization ===
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # Header
        header_layout = QHBoxLayout()
        records_label = QLabel("Recorded Elements")
        records_label.setFont(QFont("Arial", 12, QFont.Bold))
        header_layout.addWidget(records_label)
        
        self.record_count_label = QLabel("(0 items)")
        self.record_count_label.setFont(QFont("Arial", 10))
        self.record_count_label.setStyleSheet("color: #666;")
        header_layout.addWidget(self.record_count_label)
        header_layout.addStretch()
        
        # Toggle button for showing/hiding NORMAL items
        self.toggle_normal_btn = QPushButton("Hide NORMAL")
        self.toggle_normal_btn.setFont(QFont("Arial", 8))
        self.toggle_normal_btn.setCheckable(True)
        self.toggle_normal_btn.setStyleSheet("""
            QPushButton { padding: 4px 8px; background-color: #e0e0e0; border: 1px solid #bbb; border-radius: 3px; }
            QPushButton:checked { background-color: #ff9800; color: white; border: 1px solid #f57c00; }
        """)
        self.toggle_normal_btn.clicked.connect(self.toggle_normal_visibility)
        header_layout.addWidget(self.toggle_normal_btn)
        
        right_layout.addLayout(header_layout)
        
        # Scroll area for record items
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setStyleSheet("""
            QScrollArea {
                border: 1px solid #ddd;
                border-radius: 4px;
                background-color: #fafafa;
            }
        """)
        
        self.records_container = QWidget()
        self.records_layout = QVBoxLayout(self.records_container)
        self.records_layout.setAlignment(Qt.AlignTop)
        self.records_layout.setSpacing(6)
        self.records_layout.setContentsMargins(6, 6, 6, 6)
        
        scroll_area.setWidget(self.records_container)
        right_layout.addWidget(scroll_area)
        
        # Add panels to splitter
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([700, 500])
        
        main_layout.addWidget(splitter)
        
    def browse_file(self):
        """Browse and select file"""
        file_path, _ = QFileDialog.getSaveFileName(
            self, 
            "Select JSON File", 
            self.file_path_edit.text(), 
            "JSON Files (*.json)"
        )
        if file_path:
            self.file_path_edit.setText(file_path)
            self.json_file = file_path
            self.init_json_file()
    
    def init_json_file(self):
        """Initialize JSON file, ensure it exists"""
        self.json_file = self.file_path_edit.text()
        
        # Ensure directory exists
        directory = os.path.dirname(self.json_file)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
        
        # Check if file exists
        if os.path.exists(self.json_file):
            try:
                with open(self.json_file, 'r') as f:
                    data = json.load(f)
                record_count = len(data)
                key_point_count = sum(1 for r in data if r.get('key_point', False))
                self.log_message(f"Loaded file: {self.json_file}")
                self.log_message(f"  Records: {record_count} (Key: {key_point_count}, Normal: {record_count - key_point_count})")
                self.refresh_records_display()
            except json.JSONDecodeError:
                with open(self.json_file, 'w') as f:
                    json.dump([], f)
                self.log_message(f"File format error, reset: {self.json_file}")
                self.refresh_records_display()
        else:
            with open(self.json_file, 'w') as f:
                json.dump([], f)
            self.log_message(f"Created new file: {self.json_file}")
            self.refresh_records_display()
        
    def update_joint_display(self):
        """Update joint state display"""
        try:
            self.current_states = self.robot_states.get_states()
            
            # Update left arm
            for i in range(1, 7):
                joint_name = f'left_arm_joint{i}'
                value = self.current_states.get(joint_name, 0.0)
                self.left_arm_labels[joint_name].setText(f"joint{i}: {value:.4f}")
                
            # Update right arm
            for i in range(1, 7):
                joint_name = f'right_arm_joint{i}'
                value = self.current_states.get(joint_name, 0.0)
                self.right_arm_labels[joint_name].setText(f"joint{i}: {value:.4f}")
                
            # Update grippers
            left_gripper_val = self.current_states.get('left_arm_eef_gripper_joint', 0.0)
            self.left_gripper_label.setText(f"Left: {left_gripper_val:.4f}")
            
            right_gripper_val = self.current_states.get('right_arm_eef_gripper_joint', 0.0)
            self.right_gripper_label.setText(f"Right: {right_gripper_val:.4f}")
            
            # Update other joints
            head_yaw_val = self.current_states.get('head_yaw_joint', 0.0)
            self.head_yaw_label.setText(f"Head Yaw: {head_yaw_val:.4f}")
            
            head_pitch_val = self.current_states.get('head_pitch_joint', 0.0)
            self.head_pitch_label.setText(f"Head Pitch: {head_pitch_val:.4f}")
            
            slide_val = self.current_states.get('slide_joint', 0.0)
            self.slide_label.setText(f"Slide: {slide_val:.4f}")
            
        except Exception as e:
            self.log_message(f"Error updating joint states: {str(e)}")
    
    def record_raw_data(self, key_action):
        """Record raw data, append to JSON file in real-time"""
        try:
            self.json_file = self.file_path_edit.text()
            key_point = self.key_point_checkbox.isChecked()
            
            try:
                pause_value = float(self.pause_input.text())
            except ValueError:
                pause_value = 2.0
                self.pause_input.setText('2')
            
            raw_data = {
                "timestamp": time.time(),
                "head": [
                    self.current_states.get('head_yaw_joint', 0.0),
                    self.current_states.get('head_pitch_joint', 0.0)
                ],
                "lift": self.current_states.get('slide_joint', 0.0),
                "left_arm": [
                    self.current_states.get('left_arm_joint1', 0.0),
                    self.current_states.get('left_arm_joint2', 0.0),
                    self.current_states.get('left_arm_joint3', 0.0),
                    self.current_states.get('left_arm_joint4', 0.0),
                    self.current_states.get('left_arm_joint5', 0.0),
                    self.current_states.get('left_arm_joint6', 0.0)
                ],
                "right_arm": [
                    self.current_states.get('right_arm_joint1', 0.0),
                    self.current_states.get('right_arm_joint2', 0.0),
                    self.current_states.get('right_arm_joint3', 0.0),
                    self.current_states.get('right_arm_joint4', 0.0),
                    self.current_states.get('right_arm_joint5', 0.0),
                    self.current_states.get('right_arm_joint6', 0.0)
                ],
                "left_arm_eef": [0, 0, 0],
                "right_arm_eef": [0, 0, 0],
                "left_gripper": [
                    self.current_states.get('left_arm_eef_gripper_joint', 0.0)
                ],
                "right_gripper": [
                    self.current_states.get('right_arm_eef_gripper_joint', 0.0)
                ],
                "base_pose": [0, 0, 0],
                "pause": pause_value,
                "description": self.desc_input.text(),
                "key_point": key_point,
                "key_action": key_action
            }
            
            record_index = self.append_to_json_file(raw_data)
            
            point_type = "[KEY]" if key_point else "[NORMAL]"
            self.log_message(f"Recorded #{record_index} {point_type} Action: {key_action}")
            
            if key_action == 'left_arm':
                self.log_message(f"  Left Arm: {[f'{v:.4f}' for v in raw_data['left_arm']]}")
            elif key_action == 'right_arm':
                self.log_message(f"  Right Arm: {[f'{v:.4f}' for v in raw_data['right_arm']]}")
            elif key_action == 'left_gripper':
                self.log_message(f"  Left Gripper: {raw_data['left_gripper'][0]:.4f}")
            elif key_action == 'right_gripper':
                self.log_message(f"  Right Gripper: {raw_data['right_gripper'][0]:.4f}")
            elif key_action == 'pause':
                self.log_message(f"  Pause: {pause_value}s")
            
            if raw_data['description']:
                self.log_message(f"  Description: {raw_data['description']}")
            
            self.log_message(f"  Saved to {self.json_file}")
            
            # Update right panel display
            self.add_record_widget(record_index - 1, raw_data)
                
        except Exception as e:
            self.log_message(f"Error recording data: {str(e)}")
    
    def append_to_json_file(self, raw_data):
        """Append data to JSON file in real-time"""
        directory = os.path.dirname(self.json_file)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
        
        if os.path.exists(self.json_file):
            try:
                with open(self.json_file, 'r') as f:
                    data = json.load(f)
            except (json.JSONDecodeError, FileNotFoundError):
                data = []
        else:
            data = []
        
        data.append(raw_data)
        
        with open(self.json_file, 'w') as f:
            json.dump(data, f, indent=2)
        
        return len(data)
    
    def add_record_widget(self, index, record_data):
        """Add a record widget to the right panel"""
        widget = RecordItemWidget(index, record_data, self)
        self.records_layout.addWidget(widget)
        self.record_widgets.append(widget)
        self.update_key_indices()
    
    def refresh_records_display(self):
        """Refresh the entire records display"""
        # Clear existing widgets
        for widget in self.record_widgets:
            widget.deleteLater()
        self.record_widgets.clear()
        
        # Load data from file
        if os.path.exists(self.json_file):
            try:
                with open(self.json_file, 'r') as f:
                    data = json.load(f)
                
                for i, record in enumerate(data):
                    widget = RecordItemWidget(i, record, self)
                    self.records_layout.addWidget(widget)
                    self.record_widgets.append(widget)
            except (json.JSONDecodeError, FileNotFoundError):
                pass
        
        self.update_key_indices()
    
    def update_record_count(self):
        """Update the record count label"""
        count = len(self.record_widgets)
        key_count = sum(1 for w in self.record_widgets if w.record_data.get('key_point', False))
        self.record_count_label.setText(f"({count} items, {key_count} key)")
    
    def update_key_indices(self):
        """Update indices for key_point=True items only"""
        key_index = 0
        for widget in self.record_widgets:
            is_key = widget.record_data.get('key_point', False)
            if is_key:
                widget.set_key_index(key_index)
                key_index += 1
            else:
                widget.set_key_index(None)
            widget.update_display()
            # Apply visibility based on current toggle state
            if not is_key and not self.show_normal_items:
                widget.setVisible(False)
            else:
                widget.setVisible(True)
        self.update_record_count()
    
    def toggle_normal_visibility(self):
        """Toggle visibility of key_point=False items"""
        self.show_normal_items = not self.toggle_normal_btn.isChecked()
        if self.show_normal_items:
            self.toggle_normal_btn.setText("Hide NORMAL")
        else:
            self.toggle_normal_btn.setText("Show NORMAL")
        for widget in self.record_widgets:
            is_key = widget.record_data.get('key_point', False)
            if not is_key:
                widget.setVisible(self.show_normal_items)
    
    def update_record_in_file(self, index, record_data):
        """Update a specific record in the JSON file"""
        try:
            with open(self.json_file, 'r') as f:
                data = json.load(f)
            
            if 0 <= index < len(data):
                data[index] = record_data
                
                with open(self.json_file, 'w') as f:
                    json.dump(data, f, indent=2)
                
                self.log_message(f"Updated record #{index + 1} key_point to {record_data.get('key_point')}")
                self.update_record_count()
        except Exception as e:
            self.log_message(f"Error updating record: {str(e)}")
    
    def log_message(self, message):
        """Add log message"""
        self.log_text_area.append(message)
        
    def show_file_status(self):
        """Show current file status"""
        try:
            self.json_file = self.file_path_edit.text()
            
            if os.path.exists(self.json_file):
                with open(self.json_file, 'r') as f:
                    data = json.load(f)
                
                record_count = len(data)
                key_point_count = sum(1 for r in data if r.get('key_point', False))
                self.log_message(f"File Status: {self.json_file}")
                self.log_message(f"  Total Records: {record_count}")
                self.log_message(f"  Key Points: {key_point_count}, Normal Points: {record_count - key_point_count}")
                QMessageBox.information(self, "File Status", 
                    f"File: {self.json_file}\nTotal Records: {record_count}\n(Data is saved in real-time)")
            else:
                self.log_message(f"File does not exist: {self.json_file}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error checking file: {str(e)}")
            
    def load_json(self):
        """Load and refresh records display"""
        try:
            self.json_file = self.file_path_edit.text()
            
            if os.path.exists(self.json_file):
                with open(self.json_file, 'r') as f:
                    data = json.load(f)
                
                key_point_count = sum(1 for r in data if r.get('key_point', False))
                self.log_message(f"Refreshed {len(data)} records from {self.json_file}")
                self.log_message(f"  Key Points: {key_point_count}, Normal Points: {len(data) - key_point_count}")
                
                self.refresh_records_display()
            else:
                self.log_message("JSON file does not exist")
        except Exception as e:
            self.log_message(f"Error loading JSON file: {str(e)}")
            
    def clear_log(self):
        """Clear log"""
        self.log_text_area.clear()
        self.log_message("Log cleared")
    
    def create_new_file(self):
        """Create new file"""
        file_path, _ = QFileDialog.getSaveFileName(
            self, 
            "Create New JSON File", 
            "/ros2_ws/src/process/data/", 
            "JSON Files (*.json)"
        )
        if file_path:
            self.file_path_edit.setText(file_path)
            self.json_file = file_path
            
            directory = os.path.dirname(self.json_file)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
            
            with open(self.json_file, 'w') as f:
                json.dump([], f)
            
            self.log_message(f"Created new file: {self.json_file}")
            self.refresh_records_display()
    
    def clear_data(self):
        """Clear all data in current file"""
        reply = QMessageBox.question(
            self, 
            "Confirm Clear", 
            f"Are you sure you want to clear all data in {self.json_file}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                with open(self.json_file, 'w') as f:
                    json.dump([], f)
                self.log_message(f"Cleared all data: {self.json_file}")
                self.refresh_records_display()
            except Exception as e:
                self.log_message(f"Failed to clear data: {str(e)}")
        
    def closeEvent(self, event):
        """Close event"""
        self.robot_states.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    panel = JointRecorderPanel()
    panel.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
