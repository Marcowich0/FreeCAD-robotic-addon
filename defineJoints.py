import FreeCAD
import FreeCADGui

class Joint:
    def __init__(self, name, body, axis=None):
        self.name = name
        self.body = body
        self.axis = axis

class FixedJoint(Joint):
    def __init__(self, name, body):
        super().__init__(name, body)

class RevoluteJoint(Joint):
    def __init__(self, name, body, axis):
        super().__init__(name, body, axis)

class PrismaticJoint(Joint):
    def __init__(self, name, body, axis):
        super().__init__(name, body, axis)

class SelectConstraints:
    def GetResources(self):
        return {
            'Pixmap': 'path/to/icon',  # Path to an icon for the button
            'MenuText': 'List Joints',
            'ToolTip': 'Lists all joints in the active document'
        }

    def Activated(self):
        list_joints()

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None



def list_joints():
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




FreeCADGui.addCommand('SelectConstraints', SelectConstraints())





