from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QObject, QRunnable, pyqtSlot, pyqtSignal, QThreadPool
import sys
import glob
import json
from pathlib import Path
import consts.resource_paths

from resources.depthai_boards.boards_reader import DEVICES

# print(f"DEVICE_DIR: {DEVICE_DIR}")
class Ui_CalibrateSelect(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()

        # Create list of devices data
        self.device_jsons = []
        # Create "representational (title)" data
        self.device_titles = []

        # # Get all devices
        # devices = glob.glob(f'{DEVICE_DIR}/*.json')
        # # Retrieve all information
        # for dev in devices:
        #     with open(dev, 'r') as f:
        #         j = json.load(f)
        #         self.device_jsons.append(j)
        #         self.device_titles.append(j["title"])
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("CalibrateSelect", "Dialog"))

        self.setObjectName("CalibrateSelect")
        self.buttonBox = QtWidgets.QDialogButtonBox(self)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.device_dropdown = QtWidgets.QComboBox(self)
        self.device_dropdown.addItems(map(lambda d: d.get("title"), DEVICES))
        self.device_dropdown.currentTextChanged.connect(self.device_changed)

        self.variant_dropdown = QtWidgets.QComboBox(self)
        self.variant_dropdown.currentTextChanged.connect(self.variant_changed)


        self.batch_label = QtWidgets.QLabel(self)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.batch_label.setFont(font)
        self.batch_label.setObjectName("device_label")
        self.batch_label.setText(_translate("CalibrateSelect", "Device"))

        self.device_desc_label = QtWidgets.QLabel(self)
        self.device_desc_label.setObjectName("device_desc_label")

        font = QtGui.QFont()
        font.setPointSize(12)
        
        self.variant_label = QtWidgets.QLabel(self)
        self.variant_label.setGeometry(QtCore.QRect(10, 150, 77, 27))
        self.variant_label.setFont(font)
        self.variant_label.setObjectName("variant_label")
        self.variant_label.setText(_translate("CalibrateSelect", "Variant"))

        self.variant_desc_label = QtWidgets.QLabel(self)
        QtCore.QMetaObject.connectSlotsByName(self)


        # Set layout
        layout = QtWidgets.QFormLayout()
        layout.addRow(self.batch_label, self.device_dropdown)
        layout.addRow(QtWidgets.QLabel("Description"), self.device_desc_label)
        layout.addRow(self.variant_label, self.variant_dropdown)
        layout.addRow(QtWidgets.QLabel("Description"), self.variant_desc_label)
        layout.addRow(QtWidgets.QLabel("Description"), self.variant_desc_label)
        layout.addRow(self.buttonBox)
        # layout.setRowWrapPolicy(layout.RowWrapPolicy.WrapLongRows) # this causes lyout shrinkage
        self.setLayout(layout)
        # Refresh devices
        self.device_changed()

    def device_changed(self):
        self.selected_device = DEVICES[self.device_dropdown.currentIndex()]

        # Update description
        self.device_desc_label.setText(self.selected_device.get("description"))

        # Update variant dropdown
        self.variant_dropdown.clear()
        self.variant_dropdown.addItems(map(lambda v: v.get("title"), self.selected_device.get("variants")))

        self.variant_changed()

    def variant_changed(self):
        if not self.selected_device:
            return

        variants = self.selected_device.get("variants")
        self.selected_variant = variants[self.variant_dropdown.currentIndex()]
        # Update description
        self.variant_desc_label.setText(self.selected_variant.get("description"))
        print(self.selected_variant.eeprom_data)

def select_device():
    app = QtWidgets.QApplication(sys.argv)
    dialog = Ui_CalibrateSelect()
    dialog.setWindowFlags(Qt.WindowStaysOnTopHint)
    if not dialog.exec_():
        return None, None

    return dialog.eepromDataJson, dialog.test_type, dialog.selected_variant
    
    
# just for testing
if __name__ == "__main__":
    print(select_device())