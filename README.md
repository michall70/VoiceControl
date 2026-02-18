# Voice_Control_Drone
该仓库包含一个基于`ros2`和`AI Agent`的语音识别控制无人机节点

通过`Qwen AI`，实现自然语言控制仿真无人机，并允许语音输入。

目前支持的操作为：`指定位移动、指定坐标移动`

## Content
- 1.实现方式
- 2.配置环境
- 3.操作方式
- 4.效果展示
- 5.注意事项

## 1.实现方式
将ai agent作为`ros2`的一个节点，为ai添加发送位置信息的工具，实现自然语言控制无人机。

ai agent利用`langchain`框架，加入`qwen-plus`模型构建

无人机由ros2框架搭建，使用`gz_x500`模型，`gazebo`仿真，`QGroundControl`作为地面站

终端输入可选用键盘输入，或是语音输入

语音输入使用 Google 免费接口，需要代理

## 2.配置环境
```
pip install -r requirements.txt
```
还需在 `script/env.sh` 中修改文件夹和文件路径、代理配置

## 3.操作方式
Script文件夹中，包含 `VoiceControl_Drone.sh` `vcnode_only.sh`

在`terminal`执行 
```
source VoiceControl_Drone.sh
```
将启动`QGroundControl`、`gazebo`仿真环境、`ros2`环境、`代理`配置、`build ros2包`

执行如下
```
ros2 run VoiceControl VoiceControl
``` 
即可启动AI agent节点

启动后需在`px4终端`输入 
```
commander mode offboard
commander arm
```
才可以控制无人机

执行 ```source vcnode_only.sh``` 将只配置ros2环境、代理配置、build ros2包

## 4.效果展示
```
👨‍✈️ 请下达飞行指令 ( sr 语音输入， q 退出): 向北偏东30度飞行200米
🤖 AI: 无人机已成功向北偏东30度飞行200米，具体执行为：

- 向北飞行约173米，  
- 向东飞行100米。

如需进一步操作（例如调整高度、返航或飞往某绝对坐标），请随时告知！
👨‍✈️ 请下达飞行指令 ( sr 语音输入， q 退出): 飞到50,-50,20
🤖 AI: 无人机已成功飞至绝对坐标：  
- 北向：50 米  
- 东向：-50 米（即西 50 米）  
- 高度：20 米  

如需继续执行其他指令（例如悬停、拍照、返航或调整姿态），请随时告诉我！
👨‍✈️ 请下达飞行指令 ( sr 语音输入， q 退出): sr

🎤 语音助手正在启动（说“退出”以退出)
🎤 正在调节环境噪音，请保持安静 0.5 秒...
👂 正在倾听...
⌛ 正在识别语音...
👤 你说: 向南边飞行
🤖 Agent 思考中...
🤖 AI: 无人机已向南飞行 50 米。

如需继续操作（例如再向东/西/上/下移动、飞往某坐标、悬停或降落），请随时告诉我！
```
<video
src="assets/videos/Demo.mp4"
width="600"
controls
muted>
</video>

## 5.注意事项
系统可能会有麦克风增益，注意检查麦克风能否正常工作，或是调整麦克风增益

