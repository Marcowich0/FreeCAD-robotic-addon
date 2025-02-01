import FreeCAD
import FreeCADGui
from PySide import QtCore, QtGui
from main_utils import get_robot
import os

class OrientationDialog(QtGui.QDialog):
    """
    Dialog with spin boxes to edit orientation direction vector.
    Automatically normalizes the vector and validates non-zero input.
    """
    def __init__(self, robot_obj):
        super(OrientationDialog, self).__init__()
        self.robot_obj = robot_obj
        
        # Store the old orientation (to restore if user cancels)
        self.old_orientation = robot_obj.EndEffectorOrientation
        
        # Local copy to track changes
        self.current_vector = robot_obj.EndEffectorOrientation

        self.setWindowTitle("Edit Orientation Direction")
        self.layout = QtGui.QVBoxLayout(self)

        self.spin_boxes = []

        # Create vector component inputs
        components = [
            ("X Component", self.current_vector.x),
            ("Y Component", self.current_vector.y),
            ("Z Component", self.current_vector.z)
        ]

        for label, initial_val in components:
            row_widget = QtGui.QWidget()
            row_layout = QtGui.QHBoxLayout(row_widget)
            
            # Component label
            component_label = QtGui.QLabel(label)
            row_layout.addWidget(component_label)

            # Double spin box
            spin = QtGui.QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setSingleStep(0.1)
            spin.setValue(initial_val)
            spin.valueChanged.connect(self.on_vector_change)
            row_layout.addWidget(spin)
            self.spin_boxes.append(spin)

            self.layout.addWidget(row_widget)

        # Create button box
        button_box = QtGui.QDialogButtonBox(
            QtGui.QDialogButtonBox.Ok | 
            QtGui.QDialogButtonBox.Cancel |
            QtGui.QDialogButtonBox.Reset
        )
        button_box.accepted.connect(self.validate_and_accept)
        button_box.rejected.connect(self.reject)
        button_box.button(QtGui.QDialogButtonBox.Reset).clicked.connect(self.reset_to_default)
        
        self.layout.addWidget(button_box)

    def on_vector_change(self):
        """Update local vector when any component changes"""
        self.current_vector.x = self.spin_boxes[0].value()
        self.current_vector.y = self.spin_boxes[1].value()
        self.current_vector.z = self.spin_boxes[2].value()

    def validate_and_accept(self):
        """Validate vector before accepting"""
        if self.current_vector.Length != 0:
            self.current_vector.normalize()
            
        self.accept()

    def reset_to_default(self):
        """Reset to default Z-up orientation"""
        default = FreeCAD.Vector(0, 0, 0)
        self.spin_boxes[0].setValue(default.x)
        self.spin_boxes[1].setValue(default.y)
        self.spin_boxes[2].setValue(default.z)
        self.current_vector = default

    def accept(self):
        """Finalize changes"""
        # Update robot orientation through IK solver
        # You would call solve_ik here with self.current_vector
        super(OrientationDialog, self).accept()

    def reject(self):
        """Revert to original orientation"""
        # You would call solve_ik here with self.old_orientation
        super(OrientationDialog, self).reject()


class OrientationCommand:
    """
    Command that opens the OrientationDialog for the selected Robot object.
    """
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'orientation.svg'),
            'MenuText': "Edit Orientation Direction",
            'ToolTip': "Edit the end effector orientation direction vector"
        }

    def IsActive(self):
        return True if get_robot() is not None else False

    def Activated(self):
        robot = get_robot()
        if not robot:
            FreeCAD.Console.PrintError("No robot found\n")
            return

        diag = OrientationDialog(robot)
        if diag.exec_():
            # Get normalized vector and update IK
            robot.EndEffectorOrientation = diag.current_vector
            FreeCAD.Console.PrintMessage(f"New orientation: {diag.current_vector}\n")
            # Call your IK solver here:
            # solve_ik(target_pos, target_dir=target_dir)


# Register the command
FreeCADGui.addCommand("OrientationCommand", OrientationCommand())