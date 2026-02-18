import sys
import os

# 灵活的路径查找策略
# 环境变量

def get_agent_packages_dir():
    """获取 agent 虚拟环境路径"""
    if 'AGENT_PACKAGES_DIR' in os.environ:
        return os.environ['AGENT_PACKAGES_DIR']
    return None

agent_packages_dir = get_agent_packages_dir()

# Ensure the custom agent packages are importable when running via `ros2 run`.
# This prepends the directory so imports from that location are preferred.
if agent_packages_dir and os.path.isdir(agent_packages_dir) and agent_packages_dir not in sys.path:
	sys.path.insert(0, agent_packages_dir)