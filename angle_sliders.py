import FreeCAD
import FreeCADGui
from PySide import QtCore, QtGui
from main_utils import get_robot
import os

class RobotAnglesDialog(QtGui.QDialog):
    """
    Dialog with sliders to adjust the Angles property of a RobotObject.
    Reverts to old angles if user presses Cancel.
    Shows the current angle value beside each slider.
    """
    def __init__(self, robot_obj):
        super(RobotAnglesDialog, self).__init__()
        self.robot_obj = robot_obj
        
        # Store the old angles (to restore if user cancels)
        self.old_angles = list(robot_obj.Angles)
        
        # Local copy to track changes while the dialog is open
        self.angles = list(self.old_angles)

        self.setWindowTitle("Robot Angles Editor")
        self.layout = QtGui.QVBoxLayout()
        self.setLayout(self.layout)

        self.sliders = []
        self.value_labels = []  # Keep track of labels that show the current angle

        for i, angle_val in enumerate(self.angles):
            # A row with label + slider + value label
            row_widget = QtGui.QWidget()
            row_layout = QtGui.QHBoxLayout()
            row_widget.setLayout(row_layout)

            # Label for the joint name
            joint_label = QtGui.QLabel(f"Joint {i}")
            row_layout.addWidget(joint_label)

            # The slider
            slider = QtGui.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(-180)     # Adjust to your needs
            slider.setMaximum(180)
            slider.setValue(angle_val)
            slider.setSingleStep(1)
            
            # Make the slider a bit wider
            slider.setFixedWidth(300)

            # When the slider moves, update the angle array and the property
            slider.valueChanged.connect(lambda value, index=i: self.on_slider_change(index, value))
            row_layout.addWidget(slider)
            self.sliders.append(slider)

            # Label that shows the current angle value next to the slider
            value_label = QtGui.QLabel(str(angle_val))
            row_layout.addWidget(value_label)
            self.value_labels.append(value_label)

            self.layout.addWidget(row_widget)

        # Buttons: OK, Cancel
        button_box = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        self.layout.addWidget(button_box)

    def on_slider_change(self, index, value):
        """
        Called whenever a slider is moved. We update our local angles array
        and write it back to the Robot object's Angles property in real time.
        We also update the label next to the slider so it's clear which value is set.
        """
        self.angles[index] = value
        self.robot_obj.Angles = self.angles  # Real-time update
        
        # Update the label that shows the current angle
        self.value_labels[index].setText(str(value))

        # If needed to see immediate geometry updates, do:
        # FreeCAD.ActiveDocument.recompute()

    def accept(self):
        """
        Called if user presses OK. The new angles remain.
        """
        # The property is already updated in real-time, so we do nothing extra
        super(RobotAnglesDialog, self).accept()

    def reject(self):
        """
        Called if user presses Cancel. We revert to old angles.
        """
        self.robot_obj.Angles = self.old_angles
        super(RobotAnglesDialog, self).reject()

    def getAngles(self):
        """
        If the code calling this dialog wants the new angles after OK,
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
        The command is active only if we have a valid robot from get_robot().
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
