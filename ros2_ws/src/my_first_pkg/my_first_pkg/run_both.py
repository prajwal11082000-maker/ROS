import os
import sys
import subprocess

def find_setup_bat():
    candidates = []
    bases = []
    if getattr(sys, 'frozen', False):
        bases.append(os.path.dirname(sys.executable))
    bases.append(os.getcwd())
    bases.append(os.path.dirname(os.path.abspath(__file__)))
    seen = set()
    for b in bases:
        if b in seen:
            continue
        seen.add(b)
        d = b
        for _ in range(5):
            p = os.path.join(d, 'install', 'setup.bat')
            candidates.append(p)
            d = os.path.dirname(d)
    env_path = os.environ.get('ROS2_WS_SETUP')
    if env_path:
        candidates.insert(0, env_path)
    for p in candidates:
        if p and os.path.exists(p):
            return os.path.abspath(p)
    return None

def main():
    setup_bat = find_setup_bat()
    if not setup_bat:
        print('install\\setup.bat not found. Set ROS2_WS_SETUP env var to its full path.')
        sys.exit(1)
    cmd = f'call "{setup_bat}" && start "ROS2 Talker" cmd /k ros2 run my_first_pkg talker && start "ROS2 Listener" cmd /k ros2 run my_first_pkg listener'
    subprocess.run(['cmd.exe', '/c', cmd], check=False)

if __name__ == '__main__':
    main()
