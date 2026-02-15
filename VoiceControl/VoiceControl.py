import rclpy
import numpy as np
import math
import sys
import os
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus

agent_path = "/home/michall/gemini-agent"
if agent_path not in sys.path:
    sys.path.append(agent_path)
from dotenv import load_dotenv
import os
from langchain_community.chat_models import ChatTongyi
from langchain_core.tools import tool
from langchain.agents import create_agent
from langgraph.checkpoint.memory import MemorySaver
from langchain_tavily import TavilySearch
import speech_recognition as sr
import asyncio
import edge_tts
import pygame

Voice_Cmd = None #声音命令节点

# import gemini_agent.agent_drone as agent_drone

class VoiceCmd(Node):

    def __init__(self):
        super().__init__('voice_cmd') #节点名称

                # QoS profiles
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(     
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sub)
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile_sub)
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            'fmu/in/offboard_control_mode', 
            qos_profile_pub)
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            'fmu/in/trajectory_setpoint', 
            qos_profile_pub)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.trajectory_msg = TrajectorySetpoint()

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)

        #发送无人机位置指令
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            self.publisher_trajectory.publish(self.trajectory_msg)

    def send_trajectory(self, east, north, up): # 发送无人机位置指令NED
        self.trajectory_msg.position[0] = north
        self.trajectory_msg.position[1] = east
        self.trajectory_msg.position[2] = up
        self.publisher_trajectory.publish(self.trajectory_msg)

@tool
def drone_control(direction: str, distance: int) -> str:
    """
    指定无人机的飞行方向和距离。
    direction: 东、西、南、北、上、下
    distance: 飞行距离，单位米
    返回一个 TrajectorySetpoint 消息对象，包含无人机的目标位置。
    """
    global Voice_Cmd
    if Voice_Cmd == None:
        return "❌ 错误：ROS 节点未启动。"
    
    x, y, z = 0, 0, 0
    if direction == "东":
        x = 1
    elif direction == "西":
        x = -1
    elif direction == "南":
        y = -1
    elif direction == "北":
        y = 1
    elif direction == "上":
        z = 1
    elif direction == "下":
        z = -1
    else:
        return "❌ 错误：未知方向！"
    
    east = x * distance
    north = y * distance
    up = z * distance
    Voice_Cmd.send_trajectory(east, north, up)
    return f"✅ 已发送无人机飞行指令：方向 {direction}，距离 {distance} 米。"

# ros线程函数
def start_ros_thread():
    rclpy.init()
    global Voice_Cmd
    Voice_Cmd = VoiceCmd()
    rclpy.spin(Voice_Cmd)

def main(args=None):
    # 启动 ROS 线程
    ros_thread = threading.Thread(target=start_ros_thread, daemon=True)
    ros_thread.start()

    #loading
    load_dotenv(os.path.join(agent_path, ".env"))
    llm = ChatTongyi(model="qwen-plus", temperature=0)
    memory = MemorySaver()
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    tools = [drone_control]
    agent = create_agent(llm, tools, checkpointer = memory)
    config = {"configurable": {"thread_id": "user_1"}}

    # 命令循环
    while True:
        user_input = input("👨‍✈️ 请下达飞行指令 (输入 q 退出): ")
        if user_input.lower() in ['q', 'quit']:
            break
            
        # 调用 Agent
        result = agent.invoke({"messages": [("human", user_input)]}, config)
        print(f"🤖 AI: {result['messages'][-1].content}")

    Voice_Cmd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
