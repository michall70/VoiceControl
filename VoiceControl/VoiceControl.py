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

from dotenv import load_dotenv
import os
from langchain_community.chat_models import ChatTongyi
from langchain_core.tools import tool
from langchain.agents import create_agent
from langgraph.checkpoint.memory import MemorySaver

import speech_recognition as sr
import asyncio
import edge_tts
import pygame

# import queue
# import json
# import pyaudio
# from vosk import Model, KaldiRecognizer

Voice_Cmd = None #声音命令节点
recognizer = sr.Recognizer()
microphone = sr.Microphone()

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
        self.trajectory_msg.position[0] = 0
        self.trajectory_msg.position[1] = 0
        self.trajectory_msg.position[2] = -10        #默认飞行高度10米

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
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

    def send_displacement(self, north, east, down): # 发送无人机位置指令NED 相对坐标
        self.trajectory_msg.position[0] += north
        self.trajectory_msg.position[1] += east
        self.trajectory_msg.position[2] += down
        self.publisher_trajectory.publish(self.trajectory_msg)

    def send_point(self, north, east, down): # 发送无人机位置指令NED 绝对坐标
        self.trajectory_msg.position[0] = north
        self.trajectory_msg.position[1] = east
        self.trajectory_msg.position[2] = down
        self.publisher_trajectory.publish(self.trajectory_msg)

@tool
def drone_displacement(direction: str, distance: int) -> str:
    """
    指定无人机的飞行方向和距离。
    若有指定偏航角度，请先正交分解为东、西、南、北、上、下六个方向的分量，再调用此函数。
    direction: 东、西、南、北、上、下，请默认输入北
    distance: 飞行距离，单位米，请默认输入50米
    返回指令执行结果字符串
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
    up = z * distance  # ROS 中上升是负值，下降是正值
    Voice_Cmd.send_displacement(north, east, -up)
    return f"✅ 已发送无人机飞行指令：方向 {direction}，距离 {distance} 米。"

@tool
def drone_point(north: int, east: int, up: int) -> str:
    """
    指定无人机飞行到某个绝对位置坐标（<北>,<东>,<上>)。
    north: 北向坐标，单位米，请默认输入0
    east: 东向坐标，单位米，请默认输入0
    up: 上向坐标，单位米，请默认输入10（飞行高度10米）
    返回指令执行结果字符串
    """
    global Voice_Cmd
    if Voice_Cmd == None:
        return "❌ 错误：ROS 节点未启动。"

    Voice_Cmd.send_point(north, east, -up)
    return f"✅ 已发送无人机飞行指令：北 {north} 米，东 {east} 米，上 {up} 米。"

# 定义“嘴巴”函数：让 AI 说话
async def text_to_speech(text):
    output_file = "response.mp3"
    # 使用云希的声音，非常自然
    communicate = edge_tts.Communicate(text, "zh-CN-YunxiNeural")
    await communicate.save(output_file)
    
    # 播放声音
    pygame.mixer.init()
    pygame.mixer.music.load(output_file)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        await asyncio.sleep(1)
    pygame.mixer.quit()
    os.remove(output_file) # 播放完删除临时文件

#loading
def get_dotenv_dir():
    """获取 .env 文件夹路径"""
    # 1. 检查环境变量
    if 'AGENT_DOTENV_DIR' in os.environ:
        return os.environ['AGENT_DOTENV_DIR']
agent_dotenv_dir = get_dotenv_dir()
load_dotenv(os.path.join(agent_dotenv_dir, ".env"))

llm = ChatTongyi(model="qwen-plus", temperature=0)
memory = MemorySaver()
tools = [drone_displacement, drone_point]
agent = create_agent(llm, tools, checkpointer = memory)
config = {"configurable": {"thread_id": "user_1"}}
result = {}

def SpeechRecognize():
    global result
    print("\n🎤 语音助手正在启动（说“退出”以退出)")
    with sr.Microphone(device_index = None) as source:
        # 自动调节环境噪音
        print("🎤 正在调节环境噪音，请保持安静 0.5 秒...")
        recognizer.adjust_for_ambient_noise(source, duration=0.8)
        
        try:
            print("👂 正在倾听...")
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=15)
            
            # 2. 语音转文字 (使用 Google 免费接口，需要代理)
            print("⌛ 正在识别语音...")
            user_text = recognizer.recognize_google(audio, language='zh-CN')
            print(f"👤 你说: {user_text}")

            if "退出" in user_text:
                print("👋 再见！")
                return
            
            # 3. 喂给 Agent
            print("🤖 Agent 思考中...")
            result = agent.invoke({"messages": [("human", user_text)]}, config)
            
            # 4. 打印回复
            response = result["messages"][-1].content
            print(f"🤖 AI: {response}")
            # asyncio.run(text_to_speech(response))

        except sr.UnknownValueError:
            print("❓ 没听清，请再说一遍。")
        except sr.RequestError as e:
            print(f"❌ 语音服务出错（检查代理）: {e}")
        except Exception as e:
            print(f"⚠️ 发生错误: {e}")

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

    # 命令循环
    global result
    while True:
        user_input = input("👨‍✈️ 请下达飞行指令 ( sr 语音输入， q 退出): ")
        if user_input.lower() in ['q', 'quit', 'exit', '退出']:
            break
        if user_input.lower() in ['sr', 'speech', '语音']:
            SpeechRecognize()
        else:
            # 调用 Agent
            result = agent.invoke({"messages": [("human", user_input)]}, config)
            print(f"🤖 AI: {result['messages'][-1].content}")
            # asyncio.run(text_to_speech(result['messages'][-1].content))
    
    print("\n=== 🕵️‍♀️ 侦探模式：查看 AI 的完整思考过程 ===")
    all_messages = result["messages"]

    for msg in all_messages:
        # msg.type 告诉你是谁说的 (human, ai, tool)
        # msg.content 是具体内容
        print(f"\n【角色: {msg.type}】")
        print(f"内容: {msg.content}")
        
        # 如果是 AI 想要调用工具，打印一下它想调用的细节（进阶查看）
        if hasattr(msg, 'tool_calls') and msg.tool_calls:
            print(f"   (动作: AI 决定调用工具 -> {msg.tool_calls})")

    Voice_Cmd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
