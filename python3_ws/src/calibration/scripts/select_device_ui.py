from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, QRunnable, pyqtSlot, pyqtSignal, QThreadPool
import sys
import glob
import json
from pathlib import Path


DEVICE_DIR = Path(__file__).resolve().parent / 'resources/depthai-boards/batch'

print(f"DEVICE_DIR: {DEVICE_DIR}")
class Ui_CalibrateSelect(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()

        # Create list of devices data
        self.device_jsons = []
        # Create "representational (title)" data
        self.device_titles = []

        # Get all devices
        devices = glob.glob(f'{DEVICE_DIR}/*.json')
        # Retrieve all information
        for dev in devices:
            with open(dev, 'r') as f:
                j = json.load(f)
                self.device_jsons.append(j)
                self.device_titles.append(j["title"])

        self.device_dropdown = QtWidgets.QComboBox(self)
        self.device_dropdown.addItems(self.device_titles)
        self.device_dropdown.currentTextChanged.connect(self.device_changed)
        self.json_combo = QtWidgets.QComboBox(self)
        self.json_combo.currentTextChanged.connect(self.variant_changed)

        self.setObjectName("CalibrateSelect")
        self.buttonBox = QtWidgets.QDialogButtonBox(self)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.batch_label = QtWidgets.QLabel(self)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.batch_label.setFont(font)
        self.batch_label.setObjectName("device_label")

        self.device_desc_label = QtWidgets.QLabel(self)
        self.device_desc_label.setObjectName("device_desc_label")

        self.json_label = QtWidgets.QLabel(self)
        self.json_label.setGeometry(QtCore.QRect(10, 150, 77, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.json_label.setFont(font)
        self.json_label.setObjectName("json_label")

        self.variant_desc_label = QtWidgets.QLabel(self)

        QtCore.QMetaObject.connectSlotsByName(self)

        _translate = QtCore.QCoreApplication.translate
        self.batch_label.setText(_translate("CalibrateSelect", "Device"))
        self.json_label.setText(_translate("CalibrateSelect", "Variant"))
        self.setWindowTitle(_translate("CalibrateSelect", "Dialog"))

        # Set layout
        layout = QtWidgets.QFormLayout()
        layout.addRow(self.batch_label, self.device_dropdown)
        layout.addRow(QtWidgets.QLabel("Description"), self.device_desc_label)
        layout.addRow(self.json_label, self.json_combo)
        layout.addRow(QtWidgets.QLabel("Description"), self.variant_desc_label)
        layout.addRow(QtWidgets.QLabel("Description"), self.variant_desc_label)
        layout.addRow(self.buttonBox)
        # layout.setRowWrapPolicy(layout.RowWrapPolicy.WrapLongRows) # this causes lyout shrinkage
        self.setLayout(layout)

        # Refresh devices
        self.device_changed()

    def device_changed(self):
        curDevice = self.device_jsons[self.device_dropdown.currentIndex()]
        variantIndex = self.json_combo.currentIndex()

        # Create "representational (title)" data for variants
        variant_titles = []
        for variant in curDevice["variants"]:
            variant_titles.append(variant["title"])

        # Update variants combobox
        self.json_combo.clear()
        self.json_combo.addItems(variant_titles)
        # Update desc
        self.device_desc_label.setText(curDevice["description"])
        if variantIndex >= 0 and variantIndex < len(curDevice["variants"]):
            self.variant_desc_label.setText(curDevice["variants"][variantIndex]["description"])

        # Refresh variants
        self.variant_changed()


    def variant_changed(self):
        curDevice = self.device_jsons[self.device_dropdown.currentIndex()]
        variantIndex = self.json_combo.currentIndex()

        # Update desc
        if variantIndex >= 0 and variantIndex < len(curDevice["variants"]):
            self.variant_desc_label.setText(curDevice["variants"][variantIndex]["description"])
            # self.variant_desc_label = curDevice["variants"][variantIndex]["description"]

        # Load test_type, first from "device"
        self.test_type = curDevice['test_type']

        # Load eeprom data if available
        if len(curDevice['variants']) > variantIndex:
            # Load test_type, if variant also has it selected, override
            if 'test_type' in curDevice['variants'][variantIndex]:
                self.test_type = curDevice['variants'][variantIndex]["test_type"]
            self.calib_path = DEVICE_DIR / curDevice['variants'][variantIndex]["eeprom"]
            with open(self.calib_path) as jfile:
                self.eepromDataJson = json.load(jfile)

            self.selectedDeviceInfo = curDevice['variants'][variantIndex]['title']


def select_device():
	app = QtWidgets.QApplication(sys.argv)
	dialog = Ui_CalibrateSelect()
	if not dialog.exec_():
		return None

	return dialog.eepromDataJson
	
	
# just for testing
if __name__ == "__main__":
	print(select_device())