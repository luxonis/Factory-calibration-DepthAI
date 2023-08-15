import os
from os import path
from pathlib import Path


def relative_to_abs_path(relative_path):
    dirname = Path(__file__).parent
    try:
        return str((dirname / relative_path).resolve())
    except FileNotFoundError:
        return None

prefix                = relative_to_abs_path('../resources/')+"/"
device_cmd_fpath      = relative_to_abs_path('../depthai.cmd')
device_usb2_cmd_fpath = relative_to_abs_path('../depthai_usb2.cmd')
custom_calib_fpath    = relative_to_abs_path('../resources/depthai.calib')
nn_resource_path      = relative_to_abs_path('../resources/nn')+"/"
blob_fpath            = relative_to_abs_path('../resources/nn/mobilenet-ssd/mobilenet-ssd.blob')
blob_config_fpath     = relative_to_abs_path('../resources/nn/mobilenet-ssd/mobilenet-ssd.json')
factory_path          = relative_to_abs_path('../../../../../') + "/"

# if environment variable DEPTHAI_BOARDS_PRIVATE_PATH is set, use it, else use the submodule
depthai_boards_dir_path = os.environ.get("DEPTHAI_BOARDS_PRIVATE_PATH", relative_to_abs_path('../resources/depthai_boards'))
boards_dir_path       = Path(depthai_boards_dir_path) / "boards"
batch_dir_path        = Path(depthai_boards_dir_path) / "batch"

if custom_calib_fpath is not None and Path(custom_calib_fpath).exists():
    calib_fpath = custom_calib_fpath
    print("Using Custom Calibration File: depthai.calib")
else:
    calib_fpath = ''
    print("No calibration file. Using Calibration Defaults.")
