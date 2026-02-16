import sys
import os

# Ensure the custom agent packages are importable when running via `ros2 run`.
# This prepends the directory so imports from that location are preferred.
agent_path = os.path.expanduser('/home/michall/gemini-agent/agent-venv/lib/python3.12/site-packages')
if os.path.isdir(agent_path) and agent_path not in sys.path:
	sys.path.insert(0, agent_path)
