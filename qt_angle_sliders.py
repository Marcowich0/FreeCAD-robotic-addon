import FreeCAD
import FreeCADGui
from PySide import QtCore, QtGui
from main_utils import get_robot
import os

class RobotAnglesDialog(QtGui.QDialog):
    """
    Dialog with sliders and spin boxes to adjust the Angles property of a RobotObject.
    Reverts to old angles if the user presses Cancel.
    
    With the introduction of robot.AngleOffsets, the slider/spin box range for each joint is:
         [offset - 180, offset + 180]
    The displayed value is assumed to already include the offset.
    Changes made in the UI (via slider or spin box) are applied directly to robot_obj.Angles.
    The Reset button sets the value to the joint’s offset.
    """
    def __init__(self, robot_obj):
        super(RobotAnglesDialog, self).__init__()
        self.robot_obj = robot_obj
        
        # Save the original angles (assumed to include the offset)
        self.old_angles = list(robot_obj.Angles)
        # Local copy to track changes during the dialog session
        self.angles = list(self.old_angles)
        
        # Retrieve angle offsets (assumes same length as Angles); default to zeros if missing.
        self.angle_offsets = getattr(robot_obj, "AngleOffsets", [0] * len(self.angles))
        
        self.setWindowTitle("Robot Angles Editor")
        self.layout = QtGui.QVBoxLayout(self)

        self.sliders = []
        self.spin_boxes = []

        # Create a row for each joint
        for i, angle_val in enumerate(self.angles):
            offset = self.angle_offsets[i]
            row_widget = QtGui.QWidget()
            row_layout = QtGui.QHBoxLayout(row_widget)
            
            # Joint label (e.g., q1, q2, …)
            joint_label = QtGui.QLabel(f"q{i+1}")
            row_layout.addWidget(joint_label)
            
            # Slider setup: its range is relative to the offset.
            slider = QtGui.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(offset - 180, offset + 180)
            # The slider shows the value that already includes the offset.
            slider.setValue(angle_val)
            slider.setSingleStep(1)
            slider.setFixedWidth(200)
            slider.valueChanged.connect(lambda val, idx=i: self.on_slider_change(idx, val))
            row_layout.addWidget(slider)
            self.sliders.append(slider)

            # Spin box setup: mirrors the slider.
            spin = QtGui.QSpinBox()
            spin.setRange(offset - 180, offset + 180)
            spin.setValue(angle_val)
            spin.valueChanged.connect(lambda val, idx=i: self.on_spin_change(idx, val))
            row_layout.addWidget(spin)
            self.spin_boxes.append(spin)

            self.layout.addWidget(row_widget)

        # Create a button box with OK, Cancel, and a Reset button.
        button_box = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel)
        reset_button = button_box.addButton("Reset", QtGui.QDialogButtonBox.ActionRole)
        reset_button.clicked.connect(self.on_reset_clicked)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        self.layout.addWidget(button_box)

    def on_slider_change(self, index, value):
        """
        Called whenever a slider is moved.
        Applies the displayed value (which already includes the offset) directly.
        """
        self.angles[index] = value
        # Immediately update the robot object's Angles property.
        self.robot_obj.Angles = self.angles

        # Update the corresponding spin box without triggering its signals.
        self.spin_boxes[index].blockSignals(True)
        self.spin_boxes[index].setValue(value)
        self.spin_boxes[index].blockSignals(False)

        # Optional: uncomment the following line to trigger a document recompute:
        # FreeCAD.ActiveDocument.recompute()

    def on_spin_change(self, index, value):
        """
        Called when the user changes the spin box value.
        Applies the displayed value (with offset) directly.
        """
        self.angles[index] = value
        self.robot_obj.Angles = self.angles

        self.sliders[index].blockSignals(True)
        self.sliders[index].setValue(value)
        self.sliders[index].blockSignals(False)

    def on_reset_clicked(self):
        """
        Resets all joints to their default positions, i.e. their offset value.
        """
        for i in range(len(self.angles)):
            # Reset each joint's value to its offset.
            self.angles[i] = self.angle_offsets[i]
        self.robot_obj.Angles = self.angles

        # Update sliders and spin boxes accordingly.
        for i in range(len(self.angles)):
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(self.angle_offsets[i])
            self.sliders[i].blockSignals(False)

            self.spin_boxes[i].blockSignals(True)
            self.spin_boxes[i].setValue(self.angle_offsets[i])
            self.spin_boxes[i].blockSignals(False)

    def accept(self):
        """
        Pressing OK leaves the current angles in place.
        """
        super(RobotAnglesDialog, self).accept()

    def reject(self):
        """
        Pressing Cancel reverts the angles to their original values.
        """
        self.robot_obj.Angles = self.old_angles
        super(RobotAnglesDialog, self).reject()

    def getAngles(self):
        """
        Returns the current angles (which include the offset).
        """
        return self.angles


class RobotAnglesCommand:
    """
    Command that opens the RobotAnglesDialog for the selected Robot object.
    """
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'slider.svg'),
            'MenuText': "Edit Robot Angles",
            'ToolTip': "Open a dialog to edit the Angles of the selected Robot object."
        }

    def IsActive(self):
        return True if get_robot() is not None else False

    def Activated(self):
        robot = get_robot()
        if robot is None:
            FreeCAD.Console.PrintError("No robot found.\n")
            return
        diag = RobotAnglesDialog(robot)
        diag.exec_()


# Register the command with FreeCAD.
FreeCADGui.addCommand("RobotAnglesCommand", RobotAnglesCommand())
