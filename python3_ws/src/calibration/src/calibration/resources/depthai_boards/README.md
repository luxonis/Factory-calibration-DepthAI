# DepthAI Boards

This repository contains the hardware specifications for OAK devices (Hardware [docs here](https://docs.luxonis.com/projects/hardware/en/latest/)).

## Directory structure

The `batch` folder contains a file for each device type, which looks like this:
```json
{
	"title": "<board title>",
	"description": "<board description>",
	"options": {
		"bootloader": "<bootloader type>",
		"environment": "<python environment name>",
		...
	},
	"variants": [
		{
			"title": "<variant title>",
			"description": "<variant description>",
			"options": {
				"bootloader": "<bootloader type>",
				"environment": "<python environment name>",
				...
			},
			"eeprom": "<eeprom file path>",
			"board_config_file": "<board config file path>",
		}, ...
	]
}
```
> Fields that are not specified here should be considered deprecated.

`options` field is used to specify the device properties such as bootloader type, required python environment, settings used for calibration, tests that should be run, etc. They can be specified at device level or at variant level. If they are specified at variant level, they will override the device level settings.

> If a specific device needs a special treatment during calibration, flashing, etc. it should be specified in the `options` field. Code such as `if "POE" in device_name:` should be __avoided at all costs__!

`eeprom` field specifies the path to the EEPROM configuration file. EEPROM configuration files are located in the `batch/eeprom` folder and they look like this:

```json
{
    "batchName": <batch name>,
    "batchTime": <batch time in seconds since epoch>,
    "boardConf": "nIR-C30M00-00",
    "boardName": "BC2087",
    "boardRev": "R0M0E0",
    "productName": "OAK-D-LR",
    "boardCustom": "",
    "hardwareConf": "F1-FV00-BC000",
    "boardOptions": 1025,
    "version": 7
}
```

`board_config_file` field specifies the path to the baord config file. They are located in the `boards` folder. Board config file is used to specifiy camera extrinsics and other board specific settings.


### Instructions
Add this repository with
```
git submodule add https://github.com/luxonis/depthai-boards
```

Add this to your `README.md`, to let users of your project know that they need to clone this repository as well:
```
git submodule update --init --recursive
```
