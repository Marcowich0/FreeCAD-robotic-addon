import FreeCAD
import FreeCADGui
import os
import re
import RobotObject

class Link:
    def __init__(self, joint, body, edge = 0):
        self.Joint = joint
        self.Body = body
        self.Edge = edge

class SelectConstraints:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'electricMotor.svg'),
            'MenuText': 'List Joints',
            'ToolTip': 'Lists all joints in the active document'
        }

    def Activated(self):
        body = lambda ref: ref[1][0].split('.')[0]
        edge = lambda ref: int(re.search(r'\d+$', ref[1][0].split('.')[1]).group()) - 1
        joints = FreeCADGui.Selection.getSelection()

        for obj in FreeCAD.ActiveDocument.Objects:
            if hasattr(obj, 'ObjectToGround'):
                link_arr = [Link(obj, obj.ObjectToGround)] # Initialize the link array with the grounded joint to astablish the order of the rest

        for _ in joints:
            for joint in joints: # Double loop to add in the right order

                if link_arr and joint.Name not in [link.Joint.Name for link in link_arr]: # If joint is not already in link_arr
                    ref1, ref2 = body(joint.Reference1), body(joint.Reference2)
                    edge1, edge2 = edge(joint.Reference1), edge(joint.Reference2)

                    if link_arr[-1].Body.Name == ref1:
                        link_arr.append(  Link(joint, eval(f"FreeCAD.ActiveDocument.{ref2}"), edge2)  )

                    elif link_arr[-1].Body.Name == ref2:
                        link_arr.append(  Link(joint, eval(f"FreeCAD.ActiveDocument.{ref1}"), edge1)  )
                        

        link_arr.pop(0) # Remove the grounded joint (Is not a motor)
        robot = RobotObject.get_robot()
        robot.Constraints = [link.Joint for link in link_arr]
        robot.Bodies = [link.Body for link in link_arr]
        robot.Edges = [link.Edge for link in link_arr]

        RobotObject.drawDanevitHartenberg(robot)

        for i, body, edge in zip(range(len(robot.Bodies)), robot.Bodies, robot.Edges):
            print(f"Body: {body.Name}, Edge: {edge}")
            lcs = FreeCAD.ActiveDocument.addObject('PartDesign::CoordinateSystem', f'Angle_reference_LCS_{i+1}')
            original_body = body.LinkedObject
            original_body.addObject(lcs)

            circle = FreeCAD.ActiveDocument.getObject(original_body.Name).Shape.Edges[edge].Curve # Finds the circle of the constraint
            lcs.Placement.Base = circle.Center # Sets the base of the coordinate system to the center of the circle

            z_axis = circle.Axis
            temp = FreeCAD.Vector(1,0,0) if z_axis.cross(FreeCAD.Vector(1,0,0)).Length > 1e-6 else FreeCAD.Vector(0,1,0)
            x_axis = z_axis.cross(temp).normalize()
            y_axis = z_axis.cross(x_axis)

            lcs.Placement.Rotation = FreeCAD.Rotation(x_axis, y_axis, z_axis)

            lcs.ViewObject.Visibility = False
            


            

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()
        if not sel or RobotObject.get_robot() is None: 
            return False
            
        for obj in sel:
            if not (hasattr(obj, 'JointType')):
                return False
        return True


FreeCADGui.addCommand('SelectConstraints', SelectConstraints())


def get_reference(joint):
    body = lambda ref: ref[1][0].split('.')[0]
    edge = lambda ref: int(re.search(r'\d+$', ref[1][0]).group()) - 1

    body1, edge1 = body(joint.Reference1), edge(joint.Reference1)
    body2, edge2 = body(joint.Reference2), edge(joint.Reference2)

