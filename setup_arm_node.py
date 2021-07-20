
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
run_command("apt install -y python-rosdep2", sudo=True)
run_command("rm -rf py3venv", workdir=repo_root / "interbotix_ws")
run_command(f"{sys.executable} -m virtualenv py2venv --python=python2", workdir=repo_root / "interbotix_ws")
run_command("rosdep init", sudo=True, workdir=repo_root / "interbotix_ws")
run_command("catkin_make", workdir=repo_root / "interbotix_ws")
run_command("rosdep update", workdir=repo_root / "interbotix_ws")
run_command("interbotix_ws/py2venv/bin/python -m pip install rospkg pygame")
run_command(f"""echo "source {str(repo_root.absolute())}/interbotix_ws/devel/setup.bash" >> ~/.bashrc""")
run_command("""cp src/interbotix_sdk/10-interbotix-udev.rules /etc/udev/rules.d""", sudo=True)
run_command("""udevadm control --reload-rules && udevadm trigger""", sudo=True)
