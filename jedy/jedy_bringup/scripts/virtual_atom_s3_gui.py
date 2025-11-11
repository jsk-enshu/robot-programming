#!/usr/bin/env python3

"""
Virtual Atom S3 GUI Simulator

This standalone GUI tool simulates the Atom S3 device behavior without requiring Gazebo.
It provides:
- Display functionality: Subscribes to /atom_s3_additional_info (std_msgs/String) and updates GUI
- Button functionality: Publishes click count to /atom_s3_button_state (std_msgs/Int32)

Usage:
    python3 virtual_atom_s3_gui.py
    or
    ros2 run jedy_bringup virtual_atom_s3_gui.py
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel,
                             QPushButton, QVBoxLayout, QHBoxLayout)
from PyQt5.QtCore import QTimer, pyqtSignal, Qt
from PyQt5.QtGui import QFont


class RosNode(Node):
    """ROS 2 Node for handling topic communication"""

    def __init__(self):
        super().__init__('virtual_atom_s3_gui')
        self.gui = None

        # Publisher for button state
        self.button_pub = self.create_publisher(
            Int32,
            '/atom_s3_button_state',
            10
        )

        # Subscriber for display info
        self.create_subscription(
            String,
            '/atom_s3_additional_info',
            self.display_callback,
            10
        )

        self.get_logger().info('Virtual Atom S3 GUI node started')
        self.get_logger().info('Subscribing to: /atom_s3_additional_info')
        self.get_logger().info('Publishing to: /atom_s3_button_state')

    def display_callback(self, msg):
        """Callback for display info updates"""
        if self.gui:
            self.gui.update_display_signal.emit(msg.data)
            self.get_logger().debug(f'Received display update: {msg.data}')

    def publish_click(self, count):
        """Publish button click count"""
        msg = Int32()
        msg.data = count
        self.button_pub.publish(msg)
        # self.get_logger().info(f'Published button state: {count}')


class VirtualAtomS3(QWidget):
    """Virtual Atom S3 GUI Widget"""

    # Qt signal for thread-safe GUI updates
    update_display_signal = pyqtSignal(str)

    def __init__(self, ros_node, ros_timer):
        super().__init__()
        self.ros_node = ros_node
        self.ros_node.gui = self
        self.ros_timer = ros_timer  # Keep reference to ROS timer for cleanup

        # Click pattern detection (similar to OneButton.cpp)
        self.pending_clicks = 0  # Current click count in detection window
        self.total_published = 0  # Total number of clicks published (for display)
        self.click_ms = 400  # Click detection window in ms (same as OneButton default)
        self.press_ms = 1000  # Long press detection time in ms (same as OneButton default)

        # Button state
        self.current_state = 0  # 0=idle, 1-10=clicks, 11=long press, 12=long press released
        self.is_button_pressed = False
        self.press_start_time = 0

        # Timer for click pattern detection
        self.click_timer = QTimer()
        self.click_timer.setSingleShot(True)
        self.click_timer.timeout.connect(self.publish_click_pattern)

        # Timer for long press detection
        self.long_press_timer = QTimer()
        self.long_press_timer.setSingleShot(True)
        self.long_press_timer.timeout.connect(self.on_long_press_detected)

        # Timer for periodic state publishing (1Hz = 1000ms)
        self.state_publish_timer = QTimer()
        self.state_publish_timer.timeout.connect(self.publish_current_state)
        self.state_publish_timer.start(100)  # Publish every 0.1 second

        self.init_ui()

        # Connect signal to slot for thread-safe updates
        self.update_display_signal.connect(self.update_display)

    def closeEvent(self, event):
        """Handle window close event to ensure clean shutdown"""
        # Stop all timers
        self.click_timer.stop()
        self.long_press_timer.stop()
        self.state_publish_timer.stop()
        self.ros_timer.stop()

        # Cleanup ROS node
        try:
            if self.ros_node:
                self.ros_node.destroy_node()
        except Exception:
            pass

        # Shutdown rclpy if not already shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        # Accept the close event
        event.accept()

    def init_ui(self):
        """Initialize the GUI layout"""
        # Main layout
        main_layout = QVBoxLayout()

        # Title
        title_label = QLabel("Virtual Atom S3 Device")
        title_label.setAlignment(Qt.AlignCenter)
        title_font = QFont("Arial", 16, QFont.Bold)
        title_label.setFont(title_font)

        # Display area (simulates Atom S3 screen)
        self.display_label = QLabel("Waiting for /atom_s3_additional_info...")
        self.display_label.setAlignment(Qt.AlignCenter)
        self.display_label.setStyleSheet("""
            QLabel {
                background-color: #000000;
                color: #00FF00;
                font-size: 18px;
                font-family: 'Courier New', monospace;
                padding: 20px;
                border: 3px solid #333333;
                border-radius: 5px;
                min-height: 100px;
            }
        """)
        self.display_label.setWordWrap(True)

        # Button area (simulates screen button/touch)
        # Using pressed/released signals instead of clicked for long press detection
        self.click_button = QPushButton("CLICK SCREEN")
        self.click_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 15px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        self.click_button.pressed.connect(self.on_button_pressed)
        self.click_button.released.connect(self.on_button_released)

        # Status display
        status_layout = QHBoxLayout()

        # Click counter (shows published click patterns)
        self.button_state_label = QLabel("Waiting for clicks...")
        self.button_state_label.setStyleSheet("""
            QLabel {
                font-size: 14px;
                padding: 10px;
                background-color: #f0f0f0;
                border-radius: 3px;
            }
        """)

        # Topic status
        self.topic_status_label = QLabel("Status: Ready")
        self.topic_status_label.setStyleSheet("""
            QLabel {
                font-size: 14px;
                padding: 10px;
                background-color: #e3f2fd;
                border-radius: 3px;
            }
        """)

        status_layout.addWidget(self.button_state_label)
        status_layout.addWidget(self.topic_status_label)

        # Info label
        info_label = QLabel(
            "Subscribes: /atom_s3_additional_info (String)\n"
            "Publishes: /atom_s3_button_state (Int32)"
        )
        info_label.setAlignment(Qt.AlignCenter)
        info_label.setStyleSheet("""
            QLabel {
                font-size: 11px;
                color: #666666;
                padding: 5px;
            }
        """)

        # Add all widgets to main layout
        main_layout.addWidget(title_label)
        main_layout.addSpacing(10)
        main_layout.addWidget(self.display_label)
        main_layout.addSpacing(10)
        main_layout.addWidget(self.click_button)
        main_layout.addSpacing(10)
        main_layout.addLayout(status_layout)
        main_layout.addSpacing(10)
        main_layout.addWidget(info_label)

        self.setLayout(main_layout)

        # Window settings
        self.setWindowTitle("Virtual Atom S3")
        self.setGeometry(300, 300, 500, 400)
        self.setMinimumSize(400, 350)

    def on_button_pressed(self):
        """Handle button press (mouse down)"""
        from PyQt5.QtCore import QDateTime

        self.is_button_pressed = True
        self.press_start_time = QDateTime.currentMSecsSinceEpoch()

        # Start long press detection timer
        self.long_press_timer.start(self.press_ms)

        self.topic_status_label.setText("Button pressed...")

    def on_button_released(self):
        """Handle button release (mouse up)"""
        from PyQt5.QtCore import QDateTime

        if not self.is_button_pressed:
            return

        self.is_button_pressed = False
        press_duration = QDateTime.currentMSecsSinceEpoch() - self.press_start_time

        # Stop long press timer
        self.long_press_timer.stop()

        # Check if it was a long press
        if press_duration >= self.press_ms:
            # Long press released - set state to 12
            self.current_state = 12
            self.total_published += 1
            self.button_state_label.setText(f"Last: Long press released | Total: {self.total_published}")
            self.topic_status_label.setText("State: Long press released (12)")
        else:
            # It's a click - add to pattern detection
            self.pending_clicks += 1

            # Stop and restart the click detection timer
            if self.click_timer.isActive():
                self.click_timer.stop()

            # Update status to show pending clicks
            click_type = self.get_click_type_name(self.pending_clicks)
            self.topic_status_label.setText(f"Detecting: {click_type}...")

            # Start/restart timer - will publish after click_ms timeout
            self.click_timer.start(self.click_ms)

        # Reset status after 2 seconds
        QTimer.singleShot(2000, lambda: self.topic_status_label.setText("Status: Ready"))

    def on_long_press_detected(self):
        """Called when long press threshold is reached"""
        if self.is_button_pressed:
            # Set state to long press (11)
            self.current_state = 11
            self.total_published += 1
            self.button_state_label.setText(f"Last: Long press | Total: {self.total_published}")
            self.topic_status_label.setText("State: Long press (11)")

            # Cancel any pending click detection
            if self.click_timer.isActive():
                self.click_timer.stop()
            self.pending_clicks = 0

    def publish_click_pattern(self):
        """
        Publish the detected click pattern (called when timer expires).
        This mimics OneButton.cpp's click detection behavior.
        """
        if self.pending_clicks > 0:
            # Set current state to the click count (1-10)
            self.current_state = self.pending_clicks
            self.total_published += 1

            # Update display
            click_type = self.get_click_type_name(self.pending_clicks)
            self.button_state_label.setText(
                f"Last: {click_type} | Total patterns: {self.total_published}"
            )
            self.topic_status_label.setText(f"State: {self.pending_clicks} clicks")

            # Reset pending clicks
            self.pending_clicks = 0

            # Reset status after 2 seconds
            QTimer.singleShot(2000, lambda: self.topic_status_label.setText("Status: Ready"))

    def publish_current_state(self):
        """Periodically publish the current button state (called every 1 second)"""
        # Publish current state
        self.ros_node.publish_click(self.current_state)

        # Reset to idle (0) after publishing non-zero state
        # This ensures we only publish each event once
        if self.current_state != 0:
            self.current_state = 0

    def get_click_type_name(self, count):
        """Get human-readable name for click pattern"""
        if count == 1:
            return "Single-click"
        elif count == 2:
            return "Double-click"
        elif count == 3:
            return "Triple-click"
        else:
            return f"{count}-clicks"

    def update_display(self, text):
        """Update display label with received text (thread-safe via signal)"""
        self.display_label.setText(text)
        self.topic_status_label.setText("Status: Display updated!")

        # Reset status after 1 second
        QTimer.singleShot(1000, lambda: self.topic_status_label.setText("Status: Ready"))


def main(args=None):
    """Main function to run the application"""
    # Initialize QApplication first
    app = QApplication(sys.argv)

    # Initialize ROS 2
    rclpy.init(args=args)

    # Create ROS node
    node = RosNode()

    # Create QTimer to periodically call spin_once
    # This ensures ROS callbacks are processed in the GUI event loop
    ros_timer = QTimer()

    # Safe spin_once wrapper to handle shutdown gracefully
    def safe_spin_once():
        try:
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
        except Exception:
            pass

    ros_timer.timeout.connect(safe_spin_once)
    ros_timer.start(50)  # 50ms interval (20 Hz)

    # Create GUI (pass ros_timer for cleanup)
    gui = VirtualAtomS3(ros_node=node, ros_timer=ros_timer)
    gui.show()

    node.get_logger().info('Virtual Atom S3 GUI is ready!')
    node.get_logger().info('Test with:')
    node.get_logger().info('  ros2 topic pub /atom_s3_additional_info std_msgs/msg/String "data: \'Hello Atom S3\'"')
    node.get_logger().info('  ros2 topic echo /atom_s3_button_state')

    # Install signal handlers for Ctrl+C
    import signal
    def signal_handler(sig, frame):
        print("\nCtrl+C detected, shutting down...")
        gui.close()
        app.quit()

    signal.signal(signal.SIGINT, signal_handler)

    # Allow Ctrl+C to interrupt the Qt event loop
    timer = QTimer()
    timer.start(500)  # Check every 500ms for signals
    timer.timeout.connect(lambda: None)  # Empty callback to allow signal processing

    # Start Qt event loop
    exit_code = 0
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected")
    finally:
        # Cleanup (if not already done by closeEvent)
        timer.stop()
        ros_timer.stop()
        try:
            if not node._handle.is_valid():
                pass  # Already destroyed
            else:
                node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
