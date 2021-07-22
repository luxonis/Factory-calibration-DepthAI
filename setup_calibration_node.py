import subprocess
import sys
from getpass import getpass
from pathlib import Path


def run_command(command: str, workdir=None, sudo=False):
    if workdir is not None:
        print("Running command: {} inside {}".format(command, str(workdir)))
    else:
        print("Running command: {}".format(command))
    if sudo:
        command = f"echo {getpass('Please enter sudo password: ')} | sudo -S bash -c \"{command}\""
    proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdir, bufsize=1, universal_newlines=True, shell=True)

    stdout = ""
    with proc:
        for out_c in proc.stdout:
            if out_c is not None:
                print(out_c, end='')
                stdout += out_c

        stderr = proc.stderr.read()

    if proc.returncode != 0:
        print(f"Command failed with exit code {proc.returncode}!", file=sys.stderr)
        print(f"Executed: {command}", file=sys.stderr)
        print("[STDOUT]", file=sys.stderr)
        print(stdout, file=sys.stderr)
        print("[STDERR]", file=sys.stderr)
        print(stderr, file=sys.stderr)
        raise SystemExit(proc.returncode)
    print("Command ran successfully.")
    return proc, stdout, stderr


repo_root = Path(__file__).parent
run_command("rm -rf py3venv", workdir=repo_root / "python3_ws")
run_command(f"{sys.executable} -m virtualenv py3venv --python=python3", workdir=repo_root / "python3_ws")
run_command("apt-get install -y python-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libsmpeg-dev python-numpy subversion libportmidi-dev ffmpeg libswscale-dev libavformat-dev libavcodec-dev libfreetype6-dev", sudo=True)
run_command("python3_ws/py3venv/bin/python -m pip install --no-cache-dir -r requirements.txt", workdir=repo_root)
run_command("catkin_make", workdir=repo_root / "python3_ws")
run_command(f"""echo "source {str(repo_root.absolute())}/python3_ws/devel/setup.bash" >> ~/.bashrc""")
run_command("""echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules""", sudo=True)
run_command("""udevadm control --reload-rules && udevadm trigger""", sudo=True)
