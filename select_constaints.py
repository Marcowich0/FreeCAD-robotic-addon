import FreeCAD
import FreeCADGui
import os
import re
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

        lcs = FreeCAD.ActiveDocument.addObject( 'PartDesign::CoordinateSystem', f'LCS_link_{0}' )
        lcs.Placement.Rotation = FreeCAD.Rotation(FreeCAD.Vector(1,0,0), FreeCAD.Vector(0,1,0), FreeCAD.Vector(0,0,1))
        FreeCAD.ActiveDocument.getObject("RobotContainer").addObject(lcs)
        lcs_arr = [lcs]
        for i, link in enumerate(link_arr):
            lcs = FreeCAD.ActiveDocument.addObject( 'PartDesign::CoordinateSystem', f'LCS_link_{i+1}' ) # Adds coordinate system to the document

            edge_nr = int(re.search(r'\d+$', link.Joint.Reference1[1][0]).group()) - 1 # Finds edge number from reference

            print(f"ref body: {link.Body.Name}, ref edge nr {edge_nr}, joint name: {link.Joint.Name}") # debug

            circle = FreeCAD.ActiveDocument.getObject(link.Body.Name).Shape.Edges[edge_nr].Curve # Finds the circle of the constraint
            lcs.Placement.Base = circle.Center # Sets the base of the coordinate system to the center of the circle
            
            last_x = lcs_arr[-1].Placement.Rotation.multVec(FreeCAD.Vector(1,0,0))
            last_z = lcs_arr[-1].Placement.Rotation.multVec(FreeCAD.Vector(0,0,1))
            
            parallel = last_z.cross(circle.Axis).Length < 0.0001
            if parallel:
                z_axis = last_z
                x_axis = last_x

            else:
                z_axis = circle.Axis # Sets the axis of the coordinate system to the axis of the circle
                last_z.cross(z_axis)

            y_axis = z_axis.cross(x_axis)

            lcs.Placement.Rotation = FreeCAD.Rotation(x_axis, y_axis, z_axis)

            print(f"z_axis: {z_axis}, x_axis: {x_axis}, y_axis: {y_axis}") # debug
            
            
            FreeCAD.ActiveDocument.getObject("RobotContainer").addObject(lcs)
            lcs_arr.append(lcs)

        robot.CoordinateSystems = lcs_arr
            

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()
        if not sel or RobotObject.get_robot() is None: 
            return False
            
        for obj in sel:
            if not (hasattr(obj, 'JointType')):
                return False
        return True


FreeCADGui.addCommand('SelectConstraints', SelectConstraints())


