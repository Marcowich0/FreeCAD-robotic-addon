import FreeCAD
import FreeCADGui
import os
import RobotObject

class Link:
    def __init__(self, joint, body):
        self.Joint = joint
        self.Body = body

class SelectConstraints:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'electricMotor.svg'),
            'MenuText': 'List Joints',
            'ToolTip': 'Lists all joints in the active document'
        }

    def Activated(self):
        joints = FreeCADGui.Selection.getSelection()
        for obj in FreeCAD.ActiveDocument.Objects:
            if hasattr(obj, 'ObjectToGround'):
                link_arr = [Link(obj, obj.ObjectToGround)] # Initialize the link array with the grounded joint to astablish the order of the rest

        for _ in joints:
            for joint in joints: # Double loop to add in the right order

                if link_arr and joint.Name not in [link.Joint.Name for link in link_arr]: # If joint is not already in link_arr
                    ref1, ref2 = joint.Reference1[1][0].split('.')[0], joint.Reference2[1][0].split('.')[0]

                    if link_arr[-1].Body.Name in [ref1, ref2]:  # If the last joint added is connected to the current joint
                        link_arr.append( Link(joint, eval(f"FreeCAD.ActiveDocument.{ref1 if link_arr[-1].Body.Name == ref2 else ref2}")) )

        link_arr.pop(0) # Remove the grounded joint (Is not a motor)
        robot = RobotObject.get_robot()
        robot.Constraints = [link.Joint for link in link_arr]

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()
        if not sel or RobotObject.get_robot() is None: 
            return False
            
        for obj in sel:
            if not (hasattr(obj, 'JointType')):
                return False
        return True


FreeCADGui.addCommand('SelectConstraints', SelectConstraints())


