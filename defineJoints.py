import FreeCAD
import FreeCADGui
import os
import RobotObject

import json

# Base Joint class
class Joint:
    def __init__(self, joint, body, axis=None):
        self.name = joint
        self.body = body
        self.axis = axis

    #def __str__(self):
    #    return f"Name: {self.name}, Body: {self.body}, Axis: {self.axis}"

    #def __repr__(self):
    #    return f"Joint(name={self.name}, body={self.body}, axis={self.axis})"


# FixedJoint class
class FixedJoint(Joint):
    def __init__(self, name, body):
        super().__init__(name, body)

    def __str__(self):
        return f"Fixed Joint - Name: {self.name}, Body: {self.body}"

# RevoluteJoint class
class RevoluteJoint(Joint):
    def __init__(self, name, body, axis):
        super().__init__(name, body, axis)

    def __str__(self):
        return f"Revolute Joint - Name: {self.name}, Body: {self.body}, Axis: {self.axis}"

# PrismaticJoint class
class PrismaticJoint(Joint):
    def __init__(self, name, body, axis):
        super().__init__(name, body, axis)

    def __str__(self):
        return f"Prismatic Joint - Name: {self.name}, Body: {self.body}, Axis: {self.axis}"

class SelectConstraints:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'robotArm.svg'),
            'MenuText': 'List Joints',
            'ToolTip': 'Lists all joints in the active document'
        }

    def Activated(self):
        joint_arr = []
        joints = FreeCAD.ActiveDocument.getObject("Joints").OutList
        last_body_name = None

        for _ in joints:
            for joint in joints:
                
                if not joint_arr:
                    if joint.Label == "GroundedJoint": # if joint is grounded (first joint)
                        joint_arr.append(FixedJoint(joint.Name, joint.ObjectToGround.Name))

                else:
                    if joint_arr and joint.Name not in [j.name for j in joint_arr]: # if joint_arr exists and joint is not already in joint_arr
                        ref1, ref2 = joint.Reference1[1][0].split('.')[0], joint.Reference2[1][0].split('.')[0]
                        if joint_arr[-1].body in [ref1, ref2]:
                            joint_arr.append(RevoluteJoint(joint.Name, ref1 if ref1 != last_body_name else ref2, [0,0,1]))
        robot = RobotObject.initialize_robot(joint_arr)
        print("Robot initialized with the following joints:")
        for link in robot.Links:
            print(f"Name: {link.name}, Body: {link.body}, Axis: {link.axis}")

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None


FreeCADGui.addCommand('SelectConstraints', SelectConstraints())

