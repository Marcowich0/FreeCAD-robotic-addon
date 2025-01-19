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
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'robotArm.svg'),
            'MenuText': 'List Joints',
            'ToolTip': 'Lists all joints in the active document'
        }

    def Activated(self):
        link_arr = []
        joints = FreeCAD.ActiveDocument.Joints.OutList
        for _ in joints:
            for joint in joints:
                if not link_arr:
                    if joint.Label == "GroundedJoint": # if joint is grounded (first joint)
                        link_arr.append(Link(joint, eval(f"FreeCAD.ActiveDocument.{joint.ObjectToGround.Name}")))

                else:
                    if link_arr and joint.Name not in [link.Joint.Name for link in link_arr]: # if link_arr exists and joint is not already in link_arr
                        ref1, ref2 = joint.Reference1[1][0].split('.')[0], joint.Reference2[1][0].split('.')[0]
                        if link_arr[-1].Body.Name in [ref1, ref2]:
                            link_arr.append( Link(joint, eval(f"FreeCAD.ActiveDocument.{ref1 if link_arr[-1].Body.Name == ref2 else ref2}")) )


        robot = RobotObject.initialize_robot(link_arr)
        print("Robot initialized with the following joints:")
        for link in robot.Links:
            print(f"Name: {link.Joint.Name}, Body: {link.Body.Name}")

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None


FreeCADGui.addCommand('SelectConstraints', SelectConstraints())

