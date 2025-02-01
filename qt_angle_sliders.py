import FreeCAD
import FreeCADGui
from PySide import QtCore, QtGui
from main_utils import get_robot
import os

class RobotAnglesDialog(QtGui.QDialog):
    """
    Dialog with sliders and spin boxes to adjust the Angles property of a RobotObject.
    Reverts to old angles if user presses Cancel.
    Provides a 'Reset' button (alongside OK/Cancel) to set all angles to zero.
    """
    def __init__(self, robot_obj):
        super(RobotAnglesDialog, self).__init__()
        self.robot_obj = robot_obj
        
        # Store the old angles (to restore if user cancels)
        self.old_angles = list(robot_obj.Angles)
        
        # Local copy to track changes while the dialog is open
        self.angles = list(self.old_angles)

        self.setWindowTitle("Robot Angles Editor")
        self.layout = QtGui.QVBoxLayout(self)

        self.sliders = []
        self.spin_boxes = []

        # Create rows for each joint
        for i, angle_val in enumerate(self.angles):
            # A row with "Joint X" label, slider, spin box
            row_widget = QtGui.QWidget()
            row_layout = QtGui.QHBoxLayout(row_widget)
            
            # 1) Joint name
            joint_label = QtGui.QLabel(f"Joint {i}")
            row_layout.addWidget(joint_label)

            # 2) Slider
            slider = QtGui.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(angle_val)
            slider.setSingleStep(1)
            slider.setFixedWidth(200)  # Make the slider a bit wider
            # Connect slider changes
            slider.valueChanged.connect(lambda val, idx=i: self.on_slider_change(idx, val))
            row_layout.addWidget(slider)
            self.sliders.append(slider)

            # 3) Spin Box (direct number entry)
            spin = QtGui.QSpinBox()
            spin.setRange(-180, 180)
            spin.setValue(angle_val)
            spin.valueChanged.connect(lambda val, idx=i: self.on_spin_change(idx, val))
            row_layout.addWidget(spin)
            self.spin_boxes.append(spin)

            self.layout.addWidget(row_widget)

        # Create QDialogButtonBox with Ok, Cancel, and a "Reset" button
        button_box = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel)
        # Add a "Reset" button to the button box
        reset_button = button_box.addButton("Reset", QtGui.QDialogButtonBox.ActionRole)
        reset_button.clicked.connect(self.on_reset_clicked)

        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        self.layout.addWidget(button_box)

    # ----------------------------------------------------------------
    # Event Handlers
    # ----------------------------------------------------------------
    def on_slider_change(self, index, value):
        """
        Called whenever a slider is moved.
        Update our local angles array, the Robot object's angles, and the spin box.
        """
        # Update local array
        self.angles[index] = value
        # Update property in real-time
        self.robot_obj.Angles = self.angles

        # Update spin box without re-triggering signals
        self.spin_boxes[index].blockSignals(True)
        self.spin_boxes[index].setValue(value)
        self.spin_boxes[index].blockSignals(False)

        # If you need real-time recompute, uncomment:
        # FreeCAD.ActiveDocument.recompute()

    def on_spin_change(self, index, value):
        """
        Called when the user manually enters a value into the spin box.
        Syncs the slider and the Robot's angles property.
        """
        self.angles[index] = value
        self.robot_obj.Angles = self.angles

        # Update slider without re-triggering signals
        self.sliders[index].blockSignals(True)
        self.sliders[index].setValue(value)
        self.sliders[index].blockSignals(False)

    def on_reset_clicked(self):
        """
        Sets all angles to 0 immediately.
        """
        for i in range(len(self.angles)):
            self.angles[i] = 0
        self.robot_obj.Angles = self.angles

        # Update sliders and spin boxes without triggering signals
        for i in range(len(self.angles)):
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(0)
            self.sliders[i].blockSignals(False)

            self.spin_boxes[i].blockSignals(True)
            self.spin_boxes[i].setValue(0)
            self.spin_boxes[i].blockSignals(False)

    def accept(self):
        """
        Pressing OK leaves the current angles in place.
        """
        # Angles are already updated in real-time, so do nothing extra
        super(RobotAnglesDialog, self).accept()

    def reject(self):
        """
        Pressing Cancel reverts the angles to what they were on dialog open.
        """
        self.robot_obj.Angles = self.old_angles
        super(RobotAnglesDialog, self).reject()

    def getAngles(self):
        """
        If the code calling this dialog needs the new angles after OK,
        it can retrieve them here.
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
        """
        The command is active only if get_robot() returns a valid robot.
        """
        return True if get_robot() is not None else False

    def Activated(self):
        """
        Called when the user invokes the command.
        """
        robot = get_robot()
        if robot is None:
            FreeCAD.Console.PrintError("No robot found.\n")
            return

        diag = RobotAnglesDialog(robot)
        diag.exec_()


# Register the command with FreeCAD
FreeCADGui.addCommand("RobotAnglesCommand", RobotAnglesCommand())
