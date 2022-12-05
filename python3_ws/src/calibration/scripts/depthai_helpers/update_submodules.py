import os
import subprocess

def update_submodules():
	scriptDirectory = os.path.dirname(os.path.realpath(__file__))
	subprocess.check_call(['git', 'submodule', 'update', '--init', '--recursive'], cwd=scriptDirectory)