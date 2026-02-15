import os
import sys


def main():
    """Runner wrapper that ensures the local gemini-agent path is on sys.path

    This is used as the console_scripts entry point so the entry script will
    import this module first, inject the path, then call the real `main()`.
    """
    agent_path = os.path.expanduser('/home/michall/gemini-agent/agent-venv/lib/python3.12/site-packages')
    if os.path.isdir(agent_path) and agent_path not in sys.path:
        sys.path.insert(0, agent_path)

    # Import and call the real main from the package. Return its return value
    # so the console_scripts wrapper (which does `sys.exit(...)`) receives it.
    from .VoiceControl import main as vc_main
    return vc_main()
