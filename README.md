# VoiceControl - 语音控制无人机系统

🚁 一个基于ROS2和AI Agent的智能语音控制无人机系统

## 📋 目录

- [项目简介](#项目简介)
- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [安装步骤](#安装步骤)
- [使用方法](#使用方法)
- [项目结构](#项目结构)
- [配置说明](#配置说明)
- [常见问题](#常见问题)

## 🎯 项目简介

VoiceControl 是一个创新的无人机控制系统，融合了以下技术：

- **ROS2框架**：实时操作系统，与PX4飞控系统通信
- **语音识别**：使用Google Speech Recognition识别用户语音指令
- **AI Agent**：基于LLM（阿里通义千问）的智能决策引擎
- **文本转语音**：使用Edge TTS实现AI的语音反馈

用户可以通过**自然语言**（中文）下达飞行指令，系统会自动理解意图并控制无人机执行动作。

**示例对话：**
```
用户: "向北飞行50米"
AI: ✅ 已发送无人机飞行指令：方向 北，距离 50 米。
```

## ✨ 功能特性

- ✅ **语音输入**：支持中文语音命令识别
- ✅ **自然语言理解**：AI Agent理解复杂飞行指令
- ✅ **相对位移控制**：飞向指定方向和距离
- ✅ **绝对坐标控制**：飞行到指定的NED坐标
- ✅ **实时反馈**：AI语音回复执行状态
- ✅ **多种输入方式**：支持语音识别和文本输入

## 🔧 系统要求

### 硬件
- PX4兼容飞控板（如Pixhawk）
- 连接飞控的计算机（推荐Ubuntu 20.04或22.04）
- 麦克风用于语音输入
- 扬声器用于AI语音反馈

### 软件
- **Python**: 3.8+
- **ROS2**: Humble或Foxy版本
- **Internet**: 需要网络连接用于语音识别和LLM推理
- **代理/VPN**：Google Speech Recognition和阿里云服务可能需要代理

## 📦 安装步骤

### 1. 环境准备

```bash
# 确保已安装ROS2
source /opt/ros/<ROS_DISTRO>/setup.bash

# 进入工作空间
cd ~/px4_ros_com_ws
```

### 2. 安装依赖包

```bash
# 安装Python依赖
pip install -r requirements.txt

# 或手动安装各个包
pip install langchain langchain-community langgraph
pip install SpeechRecognition
pip install edge-tts
pip install pygame
pip install python-dotenv
pip install PyAudio  # 用于麦克风输入
```

### 3. 配置API密钥

创建 `.env` 文件（在项目根目录或通过 `AGENT_DOTENV_DIR` 环境变量指定）：

```env
# .env 文件示例
# 阿里通义千问API密钥（可选配置）
TONGYI_API_KEY=your_api_key_here
```

### 4. 编译ROS2包

```bash
# 进入工作空间根目录
cd ~/px4_ros_com_ws

# 编译该包
colcon build --packages-select VoiceControl

# 加载环境变量
source install/setup.bash
```

## 🚀 使用方法

### 启动系统

```bash
# 终端1：启动PX4仿真环境或连接真实飞控
# 终端2：启动ROS节点
ros2 run VoiceControl VoiceControl
```

### 交互式命令

启动后，你会看到提示符：

```
👨‍✈️ 请下达飞行指令 ( sr 语音输入， q 退出): 
```

**命令选项：**

| 命令 | 说明 | 示例 |
|------|------|------|
| `q` / `exit` | 退出程序 | `q` |
| `sr` / `语音` | 启动语音识别 | `sr` |
| 任何文本 | 发送给AI的飞行指令 | `向北飞行50米` |

### 语音控制示例

```
👨‍✈️ 请下达飞行指令 ( sr 语音输入， q 退出): sr
🎤 语音助手正在启动（说"退出"以退出)
🎤 正在调节环境噪音，请保持安静 0.5 秒...
👂 正在倾听...
👤 你说: 向北飞行50米
🤖 Agent 思考中...
🤖 AI: ✅ 已发送无人机飞行指令：方向 北，距离 50 米。
```

### 文本控制示例

```
👨‍✈️ 请下达飞行指令 ( sr 语音输入， q 退出): 向上上升5米
🤖 AI: ✅ 已发送无人机飞行指令：方向 上，距离 5 米。
```

## 📁 项目结构

```
VoiceControl/
├── README.md                          # 项目说明文档
├── LICENSE                            # Apache 2.0 许可证
├── package.xml                        # ROS2包配置文件
├── setup.py                           # Python安装脚本
├── .env                               # 环境变量配置（不上传至Git）
│
├── VoiceControl/                      # 主程序包
│   ├── __init__.py
│   ├── VoiceControl.py               # 主程序入口
│   ├── offboard_control_dk.py        # 无人机控制模块（开发版）
│   └── offboard_control_tst.py       # 无人机控制模块（测试版）
│
├── model/                             # 语音识别模型（本地识别器）
│   ├── am/                            # 声学模型
│   ├── graph/                         # 解码图
│   ├── ivector/                       # i-vector提取器
│   └── conf/                          # 配置文件
│
├── script/                            # 辅助脚本
│   ├── env.sh                         # 环境加载脚本
│   ├── Start.sh                       # 启动脚本
│   ├── VoiceControl_Drone.sh         # 无人机模式启动脚本
│   └── Setup_ROS2Offboard.sh         # ROS2离线控制设置
│
├── resource/                          # 资源文件
│   └── VoiceControl/                  # 启动脚本资源
│
└── test/                              # 测试文件
    ├── test_copyright.py              # 版权检查
    ├── test_flake8.py                 # 代码风格检查
    └── test_pep257.py                 # 文档字符串检查
```

## ⚙️ 配置说明

### ROS2 QoS 配置

程序中预定义了两套QoS（服务质量）配置：

```python
# 发布者配置：强可靠性 + 持久数据
qos_profile_pub = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# 订阅者配置：尽力交付 + 易失数据
qos_profile_sub = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)
```

### 飞行参数调整

在 `VoiceControl` 类的 `__init__` 方法中修改默认飞行高度：

```python
# 默认飞行高度：10米（负值表示向下）
self.trajectory_msg.position[2] = -10
```

### 坐标系说明

系统使用 **NED坐标系**（北-东-下）：

| 轴向 | 正方向 | 说明 |
|------|--------|------|
| North | 正值 | 向北飞行 |
| East | 正值 | 向东飞行 |
| Down | 正值 | 向下飞行 |

## ❓ 常见问题

### Q: 语音识别不工作
**A:** 检查以下几点：
1. 确保麦克风已连接并正常工作：`arecord -l` 查看设备列表
2. 检查网络连接（Google Speech Recognition需要网络）
3. 尝试配置HTTP代理以访问Google服务
4. 增加调节环境噪音的时间：修改 `duration=0.8` 参数

### Q: AI反应很慢
**A:** 可能的原因和解决方法：
1. 网络延迟：使用国内API服务可能会更快
2. 模型加载：首次运行需要加载LLM模型
3. 检查系统资源：`top` 命令查看CPU和内存占用

### Q: 无人机不响应指令
**A:** 检查以下条件：
1. 飞控是否成功进入Offboard模式
2. 无人机是否已解锁（Arming）
3. 检查ROS话题是否正常通信：`ros2 topic list` 和 `ros2 topic echo`
4. 查看飞控日志了解具体错误

### Q: ModuleNotFoundError: 缺少某个模块
**A:** 重新安装依赖包：
```bash
pip install --upgrade -r requirements.txt
```

### Q: 如何在实际飞控上测试？
**A:** 
1. 用USB连接飞控到计算机
2. 启动PX4固件和通信中间件（如MAVProxy或xrce-dds）
3. 运行VoiceControl节点

## 📚 进阶开发

### 添加新的飞行指令

在 `VoiceControl.py` 中添加新工具函数：

```python
@tool
def your_new_command(param1: str) -> str:
    """
    你的命令描述
    param1: 参数说明
    """
    global Voice_Cmd
    # 实现你的功能
    return "✅ 执行成功"

# 将工具添加到Agent
tools = [drone_displacement, drone_point, your_new_command]
```

### 集成本地语音模型

项目已包含本地Kaldi语音模型（在 `model/` 目录），可替代Google Speech Recognition以避免网络依赖。

## 📄 许可证

本项目采用 **Apache License 2.0** 许可证。详见 [LICENSE](LICENSE) 文件。

## 👤 维护者

**michall** - michall@gmail.com

## 🤝 贡献指南

欢迎提出Issue和Pull Request！

---

**最后更新**: 2026年2月

⭐ 如果这个项目对你有帮助，请点个Star！
