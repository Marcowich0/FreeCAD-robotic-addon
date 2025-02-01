import FreeCAD
import FreeCADGui
from PySide import QtCore, QtGui
import os
from main_utils import get_robot  # Assumes this function exists and returns the robot object

from inverse_kinematics import solve_ik

class RobotEndEffectorDialog(QtGui.QDialog):
    """
    A dialog to move the robot's end effector with arrow buttons and a configurable step size.
    
    Clicking a button will move the current EndEffectorGlobal position by the specified step (in mm)
    and call solve_ik(new_position).
    """
    def __init__(self, robot_obj, step=1.0):
        super(RobotEndEffectorDialog, self).__init__()
        self.robot_obj = robot_obj
        self.step = step  # Default step in mm
        
        self.setWindowTitle("Move End Effector")
        self.layout = QtGui.QVBoxLayout(self)
        
        # --- Step Size Control ---
        step_layout = QtGui.QHBoxLayout()
        step_label = QtGui.QLabel("Step size (mm):")
        self.step_spin = QtGui.QDoubleSpinBox()
        self.step_spin.setRange(0.1, 1000.0)
        self.step_spin.setSingleStep(0.1)
        self.step_spin.setValue(self.step)
        self.step_spin.valueChanged.connect(self.on_step_changed)
        step_layout.addWidget(step_label)
        step_layout.addWidget(self.step_spin)
        self.layout.addLayout(step_layout)
        
        # --- X-Y Movement Buttons ---
        grid = QtGui.QGridLayout()
        
        # Create buttons for forward, backward, left, and right moves.
        # Here, "Forward" moves in +Y, "Backward" in -Y,
        # "Right" moves in +X, and "Left" moves in -X.
        self.btn_forward = QtGui.QPushButton("↑")
        self.btn_backward = QtGui.QPushButton("↓")
        self.btn_left = QtGui.QPushButton("←")
        self.btn_right = QtGui.QPushButton("→")
        
        # Connect X-Y movement signals.
        self.btn_forward.clicked.connect(lambda: self.move_effector(0, self.step, 0))
        self.btn_backward.clicked.connect(lambda: self.move_effector(0, -self.step, 0))
        self.btn_left.clicked.connect(lambda: self.move_effector(-self.step, 0, 0))
        self.btn_right.clicked.connect(lambda: self.move_effector(self.step, 0, 0))
        
        # Arrange the buttons in a grid.
        # Layout:
        #      [   ] [↑]   [   ]
        #      [←]  [   ] [→]
        #      [   ] [↓]   [   ]
        grid.addWidget(self.btn_forward, 0, 1)
        grid.addWidget(self.btn_left,    1, 0)
        grid.addWidget(self.btn_right,   1, 2)
        grid.addWidget(self.btn_backward,1, 1)
        self.layout.addLayout(grid)
        
        # --- Z Movement Buttons ---
        z_layout = QtGui.QHBoxLayout()
        self.btn_z_up = QtGui.QPushButton("Z Up")
        self.btn_z_down = QtGui.QPushButton("Z Down")
        self.btn_z_up.clicked.connect(lambda: self.move_effector(0, 0, self.step))
        self.btn_z_down.clicked.connect(lambda: self.move_effector(0, 0, -self.step))
        z_layout.addWidget(self.btn_z_up)
        z_layout.addWidget(self.btn_z_down)
        self.layout.addLayout(z_layout)
        
        # --- OK/Cancel Buttons ---
        button_box = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        self.layout.addWidget(button_box)
    
    def on_step_changed(self, value):
        """
        Update the step size when the spin box value changes.
        """
        self.step = value
    
    def move_effector(self, dx, dy, dz):
        """
        Adjust the end effector position by the given deltas and call solve_ik().
        """
        # Get the current position of the end effector.
        current_pos = self.robot_obj.EndEffectorGlobal  # Assumed to be a FreeCAD.Vector
        
        # Compute the new position.
        new_pos = FreeCAD.Vector(current_pos.x + dx,
                                 current_pos.y + dy,
                                 current_pos.z + dz)
        
        # Call the inverse kinematics solver to move the robot.
        solve_ik(new_pos)
        
        # Optionally, recompute the document if needed.
        FreeCAD.ActiveDocument.recompute()

class MoveEndEffectorCommand:
    """
    Command that opens the RobotEndEffectorDialog to move the end effector.
    """
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'controller.svg'),
            'MenuText': "Move End Effector",
            'ToolTip': "Open a dialog with arrow buttons to adjust the end effector's target position."
        }
    
    def IsActive(self):
        """
        Activate only if a valid robot exists.
        """
        return True if get_robot() is not None else False
    
    def Activated(self):
        """
        Create and show the dialog when the command is activated.
        """
        robot = get_robot()
        if robot is None:
            FreeCAD.Console.PrintError("No robot found.\n")
            return
        
        diag = RobotEndEffectorDialog(robot)
        diag.exec_()

# Register the command with FreeCAD.
FreeCADGui.addCommand("MoveEndEffectorCommand", MoveEndEffectorCommand())
