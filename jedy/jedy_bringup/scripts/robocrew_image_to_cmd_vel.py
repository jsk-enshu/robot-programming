#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import TwistStamped
from rviz_2d_overlay_msgs.msg import OverlayText
from cv_bridge import CvBridge
import cv2
import numpy as np
import base64
import math

from langchain_core.tools import tool
from langchain_core.messages import HumanMessage, SystemMessage, ToolMessage
from langchain.chat_models import init_chat_model
from dotenv import find_dotenv, load_dotenv

load_dotenv(find_dotenv())


def horizontal_angle_grid(image, h_fov=120, center_angle=0):
    """Draw horizontal angle markers on the bottom of the image."""
    height, width = image.shape[:2]
    yellow = (0, 255, 255)
    orange = (0, 100, 255)
    y_pos = 25
    mark_len_angle = 10

    # Draw baseline
    cv2.line(image, (0, y_pos), (width, y_pos), yellow, 2)

    # Generate markers every 5° within visible range
    nr_of_marks = int((h_fov / 2) // mark_len_angle * 2 + 1)
    pixels_per_mark = width / h_fov * mark_len_angle
    start_pixel = (width - (nr_of_marks - 1) * pixels_per_mark) / 2
    start_angle = (-h_fov / 2 + center_angle)
    start_angle = mark_len_angle * math.trunc(start_angle / mark_len_angle)

    for mark_number in range(nr_of_marks):
        x = int(start_pixel + mark_number * pixels_per_mark)
        angle = start_angle + mark_number * mark_len_angle
        cv2.line(image, (x, y_pos - 10), (x, y_pos + 10), yellow, 2)
        cv2.putText(image, f"{angle}", (x - 15, y_pos + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, yellow, 2)

    # put right/left text
    cv2.putText(image, "<=LEFT", (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, orange, 2)
    cv2.putText(image, "RIGHT=>", (width - 145, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, yellow, 2)
    return image


class RoboCrewImageToCmdVel(Node):
    def __init__(self):
        super().__init__('robocrew_image_to_cmd_vel')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/mecanum_drive_controller/reference')
        self.declare_parameter('image_topic', '/camera/color/image_rect_raw')
        self.declare_parameter('prompt_topic', '/robocrew/prompt')
        self.declare_parameter('model', 'google_genai:gemini-robotics-er-1.5-preview')
        self.declare_parameter('camera_fov', 120)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.3)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        image_topic = self.get_parameter('image_topic').value
        prompt_topic = self.get_parameter('prompt_topic').value
        model = self.get_parameter('model').value
        self.camera_fov = self.get_parameter('camera_fov').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Publisher and subscribers
        self.publisher_ = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.status_publisher = self.create_publisher(String, '/robocrew/status', 10)
        self.overlay_publisher = self.create_publisher(OverlayText, '/robocrew/overlay', 10)
        self.image_subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.prompt_subscription = self.create_subscription(
            String, prompt_topic, self.prompt_callback, 10)

        # CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()

        # Latest image and prompt
        self.latest_image = None
        self.current_prompt = "Find green object"

        # Task state
        self.task_active = True
        self.task_completed = False
        self.prompt_changed = False  # Flag to indicate new prompt received

        # LLM setup with tools
        self.setup_llm(model)

        # Message history
        self.system_message = SystemMessage(content="You are a mobile robot. Control the robot's movement based on camera input and task instructions.")
        self.message_history = [self.system_message]

        # Timer for periodic control loop
        self.timer = self.create_timer(2.0, self.control_loop)

        self.get_logger().info('RoboCrew Image to CmdVel node initialized')

    def setup_llm(self, model):
        """Setup LLM with movement tools"""
        # Create movement tools that publish cmd_vel
        @tool
        def move_forward(duration_seconds: float = 1.0) -> str:
            """Move the robot forward for a specified duration in seconds"""
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.twist.linear.x = self.linear_speed
            self.publisher_.publish(cmd_vel)
            self.get_logger().info(f'Moving forward for {duration_seconds}s')
            return f"Moving forward for {duration_seconds} seconds"

        @tool
        def move_backward(duration_seconds: float = 1.0) -> str:
            """Move the robot backward for a specified duration in seconds"""
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.twist.linear.x = -self.linear_speed
            self.publisher_.publish(cmd_vel)
            self.get_logger().info(f'Moving backward for {duration_seconds}s')
            return f"Moving backward for {duration_seconds} seconds"

        @tool
        def turn_left(duration_seconds: float = 1.0) -> str:
            """Turn the robot left for a specified duration in seconds"""
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.twist.angular.z = self.angular_speed
            self.publisher_.publish(cmd_vel)
            self.get_logger().info(f'Turning left for {duration_seconds}s')
            return f"Turning left for {duration_seconds} seconds"

        @tool
        def turn_right(duration_seconds: float = 1.0) -> str:
            """Turn the robot right for a specified duration in seconds"""
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.twist.angular.z = -self.angular_speed
            self.publisher_.publish(cmd_vel)
            self.get_logger().info(f'Turning right for {duration_seconds}s')
            return f"Turning right for {duration_seconds} seconds"

        @tool
        def stop() -> str:
            """Stop the robot"""
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.twist.linear.x = 0.0
            cmd_vel.twist.angular.z = 0.0
            self.publisher_.publish(cmd_vel)
            self.get_logger().info('Stopping')
            return "Stopped"

        self.tools = [move_forward, move_backward, turn_left, turn_right, stop]
        self.tool_name_to_tool = {tool.name: tool for tool in self.tools}

        # Initialize LLM
        llm = init_chat_model(model)
        self.llm = llm.bind_tools(self.tools, parallel_tool_calls=False)

    def image_callback(self, msg):
        """Callback for receiving images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def prompt_callback(self, msg):
        """Callback for receiving prompts"""
        new_prompt = msg.data

        # Check if prompt actually changed
        if new_prompt != self.current_prompt:
            self.get_logger().info(f'New prompt received, interrupting current task')
            self.get_logger().info(f'Old: {self.current_prompt}')
            self.get_logger().info(f'New: {new_prompt}')

            # Immediately stop the robot
            self.send_stop_command()

            # Update prompt and reset state
            self.current_prompt = new_prompt
            self.task_active = True
            self.task_completed = False
            self.prompt_changed = True

            # Reset message history for new task
            self.message_history = [self.system_message]

            self.publish_status("ACTIVE", f"New task started: {self.current_prompt}")
        else:
            self.get_logger().info(f'Same prompt received, ignoring')


    def process_image_with_grid(self, image):
        """Process image with angle grid overlay"""
        if image is None:
            return None

        # Add horizontal angle grid
        image_with_grid = horizontal_angle_grid(image, h_fov=self.camera_fov)

        # Encode to JPEG
        _, buffer = cv2.imencode('.jpg', image_with_grid)
        return buffer.tobytes()

    def invoke_tool(self, tool_call):
        """Execute a tool call"""
        requested_tool = self.tool_name_to_tool[tool_call["name"]]
        args = tool_call["args"]
        tool_output = requested_tool.invoke(args)
        return ToolMessage(tool_output, tool_call_id=tool_call["id"])

    def send_stop_command(self):
        """Send stop command to robot"""
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.linear.y = 0.0
        cmd_vel.twist.angular.z = 0.0
        self.publisher_.publish(cmd_vel)

    def publish_status(self, state, message):
        """Publish task status"""
        status_msg = String()
        status_msg.data = f"[{state}] {message}"
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f'Status: {status_msg.data}')

        # Also publish overlay for RViz
        self.publish_overlay(state, message)

    def publish_overlay(self, state, message):
        """Publish overlay text for RViz display"""
        overlay = OverlayText()
        overlay.action = OverlayText.ADD

        # Position in top-left corner
        overlay.width = 500
        overlay.height = 200
        overlay.horizontal_distance = 10
        overlay.vertical_distance = 10
        overlay.horizontal_alignment = OverlayText.LEFT
        overlay.vertical_alignment = OverlayText.TOP

        # Background color based on state
        overlay.bg_color = ColorRGBA()
        if state == "COMPLETED":
            overlay.bg_color.r = 0.0
            overlay.bg_color.g = 0.8
            overlay.bg_color.b = 0.0
            overlay.bg_color.a = 0.8
        elif state == "ERROR":
            overlay.bg_color.r = 0.8
            overlay.bg_color.g = 0.0
            overlay.bg_color.b = 0.0
            overlay.bg_color.a = 0.8
        elif state == "PROCESSING" or state == "EXECUTING":
            overlay.bg_color.r = 0.0
            overlay.bg_color.g = 0.5
            overlay.bg_color.b = 0.8
            overlay.bg_color.a = 0.8
        elif state == "WAITING" or state == "IDLE":
            overlay.bg_color.r = 0.5
            overlay.bg_color.g = 0.5
            overlay.bg_color.b = 0.5
            overlay.bg_color.a = 0.8
        else:  # ACTIVE
            overlay.bg_color.r = 0.0
            overlay.bg_color.g = 0.7
            overlay.bg_color.b = 0.0
            overlay.bg_color.a = 0.8

        # Text style
        overlay.line_width = 2
        overlay.text_size = 14.0
        overlay.font = "DejaVu Sans Mono"
        overlay.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        # Text content
        overlay.text = f"RoboCrew Status\n"
        overlay.text += f"================\n"
        overlay.text += f"State: {state}\n"
        overlay.text += f"Task: {self.current_prompt}\n"
        overlay.text += f"----------------\n"
        overlay.text += f"{message}"

        self.overlay_publisher.publish(overlay)

    def control_loop(self):
        """Main control loop that processes image and prompt with LLM"""
        # Check if prompt changed - if so, skip this cycle to allow immediate stop
        if self.prompt_changed:
            self.prompt_changed = False
            self.get_logger().info('Prompt changed, skipping this control cycle')
            return

        # If task is completed, skip LLM call and just send stop
        if self.task_completed:
            self.send_stop_command()
            return

        if self.latest_image is None:
            self.get_logger().warn('No image received yet, skipping control loop')
            self.send_stop_command()
            self.publish_status("WAITING", "Waiting for camera image")
            return

        if not self.task_active:
            self.send_stop_command()
            self.publish_status("IDLE", "Task inactive, awaiting new prompt")
            return

        try:
            # Process image
            image_bytes = self.process_image_with_grid(self.latest_image)
            if image_bytes is None:
                self.send_stop_command()
                return

            image_base64 = base64.b64encode(image_bytes).decode('utf-8')

            # Create message with image and prompt
            message = HumanMessage(
                content=[
                    {"type": "text", "text": "Here is the current view from the robot's camera. Use it to understand the current situation."},
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}
                    },
                    {"type": "text", "text": f"Your task is: '{self.current_prompt}'"}
                ]
            )

            self.message_history.append(message)

            # Get LLM response
            self.publish_status("PROCESSING", f"Processing image for task: {self.current_prompt}")
            response = self.llm.invoke(self.message_history)
            self.get_logger().info(f'LLM response: {response.content}')
            self.get_logger().info(f'Tool calls: {response.tool_calls}')

            self.message_history.append(response)

            # Check if LLM indicates task completion
            if response.content:
                content_str = str(response.content).lower()
                if any(keyword in content_str for keyword in ['found', 'completed', 'finished', 'done']):
                    self.task_completed = True
                    self.task_active = False
                    self.publish_status("COMPLETED", f"Task completed: {response.content}")
                    self.send_stop_command()
                    return

            # Execute tool calls
            if len(response.tool_calls) > 0:
                for tool_call in response.tool_calls:
                    tool_response = self.invoke_tool(tool_call)
                    self.message_history.append(tool_response)
                    # Publish action being taken
                    action_name = tool_call['name'].replace('_', ' ').title()
                    self.publish_status("EXECUTING", f"{action_name}: {tool_call.get('args', {})}")

                    # Check if stop tool was called - if so, mark task as completed
                    if tool_call['name'] == 'stop':
                        self.task_completed = True
                        self.task_active = False
                        self.publish_status("COMPLETED", "Task completed: Robot stopped as requested")
                        self.get_logger().info('Stop tool called, marking task as completed')
                        return
            else:
                # No tool calls - explicitly send stop command
                self.get_logger().info('No tool calls from LLM, sending stop command')
                self.send_stop_command()
                self.publish_status("IDLE", "No action needed")

            # Keep history manageable
            # Keep only: system message + last complete exchange (human + ai + tools)
            # This ensures we always have the correct message order for Gemini
            if len(self.message_history) > 6:  # system + 1 full exchange (human + ai + tool)
                # Find the last HumanMessage
                human_indices = [i for i, msg in enumerate(self.message_history) if msg.type == "human"]
                if len(human_indices) >= 2:
                    # Keep system message and everything from the second-to-last human message
                    last_exchange_start = human_indices[-2]
                    self.message_history = [self.system_message] + self.message_history[last_exchange_start:]

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
            # Send stop command on error
            self.send_stop_command()
            # Publish error status
            self.publish_status("ERROR", f"Error occurred: {str(e)}")
            # Reset message history to avoid corrupted state
            self.message_history = [self.system_message]


def main(args=None):
    rclpy.init(args=args)
    node = RoboCrewImageToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
