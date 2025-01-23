import FreeCAD
import FreeCADGui
import os
import re
from main_utils import get_robot


class DrawDanevitHartenbergCommand:
    """Command to draw the robot using Denavit-Hartenberg parameters."""

    def __init__(self):
        pass

    def GetResources(self):
        """Returns the resources associated with the command."""
        return {
            'MenuText': 'Draw Denavit-Hartenberg',
            'ToolTip': 'Draw the robot using Denavit-Hartenberg parameters',
            'Pixmap': 'path/to/icon.svg'  # Provide the path to your icon
        }

    def Activated(self):
        """Called when the command is activated (e.g., button pressed)."""
        try:
            drawDanevitHartenberg()
            FreeCAD.Console.PrintMessage("Danevit-Hartenberg drawing completed successfully.\n")
        except Exception as e:
            FreeCAD.Console.PrintError(f"Error: {e}\n")

    def IsActive(self):
        """Determines if the command is active."""
        # You can add conditions here if needed
        return True if get_robot() != None else False

FreeCADGui.addCommand('DrawDanevitHartenbergCommand', DrawDanevitHartenbergCommand())



def drawDanevitHartenberg():
    doc = FreeCAD.ActiveDocument
    obj = get_robot()
    if not obj.CoordinateSystems:
        """Draw the robot using the Denavit-Hartenberg parameters."""
        lcs = doc.addObject( 'PartDesign::CoordinateSystem', f'LCS_link_{0}' )
        obj.Base.LinkedObject.addObject(lcs)
    else:
        lcs = obj.CoordinateSystems[0]

    lcs.Placement.Rotation = FreeCAD.Rotation(FreeCAD.Vector(1,0,0), FreeCAD.Vector(0,1,0), FreeCAD.Vector(0,0,1))
    lcs_arr = [lcs]

        
    for i, body, edge in zip(range(len(obj.Constraints)), obj.Bodies, obj.Edges):
        if not obj.CoordinateSystems:
            print(f" -- Creating LCS for joint {i+1}") # debug
            lcs = doc.addObject( 'PartDesign::CoordinateSystem', f'LCS_link_{i+1}' ) # Adds coordinate system to the document
            obj.Base.LinkedObject.addObject(lcs)
        else:
            lcs = obj.CoordinateSystems[i+1]

        edge_nr = edge

        print(f"ref body: {body.Name}, ref edge nr {edge_nr}, joint name: {body.Name}") # debug

        circle = doc.getObject(body.Name).Shape.Edges[edge_nr].Curve # Finds the circle of the constraint
        lcs.Placement.Base = circle.Center # Sets the base of the coordinate system to the center of the circle
        
        last_x = lcs_arr[-1].Placement.Rotation.multVec(FreeCAD.Vector(1,0,0))
        last_z = lcs_arr[-1].Placement.Rotation.multVec(FreeCAD.Vector(0,0,1))
        
        parallel = last_z.cross(circle.Axis).Length < 1e-6 # Checks if the circle axis is parallel to the last z-axis
        print(f"parallel: {parallel}") # debug
        if parallel:
            z_axis = last_z
            x_axis = last_x

        else:
            z_axis = circle.Axis # Sets the axis of the coordinate system to the axis of the circle
            x_axis = last_z.cross(z_axis)

        y_axis = z_axis.cross(x_axis)

        lcs.Placement.Rotation = FreeCAD.Rotation(x_axis, y_axis, z_axis)
        lcs_arr.append(lcs)

        if not any(plane.Name == f'plane_on_DH_coordinates_{i+1}' for plane in doc.Objects):
            datum_plane = doc.addObject('PartDesign::Plane', f'plane_on_DH_coordinates_{i+1}')
            obj.Base.LinkedObject.addObject(datum_plane)
            datum_plane.AttachmentOffset = FreeCAD.Placement(
                FreeCAD.Vector(0.0, 0.0, 0.0),  # Base position of the plane
                FreeCAD.Rotation(FreeCAD.Vector(0, 1, 0), 0.0)  # No additional rotation
            )
            datum_plane.MapReversed = False
            datum_plane.AttachmentSupport = [(lcs, '')]  # Attach to the LCS
            datum_plane.MapPathParameter = 0.0
            datum_plane.MapMode = 'ObjectXZ'  # Align to the X-Z plane of the LCS
            datum_plane.recompute()
            datum_plane.ViewObject.Visibility = False


        print(f"z_axis: {z_axis}, x_axis: {x_axis}, y_axis: {y_axis}") # debug
        print(f"length of z_axis: {round(z_axis.Length, 3)}, length of x_axis: {round(x_axis.Length, 3)}, length of y_axis: {round(y_axis.Length, 3)}")
        print(" -- ")
        

    obj.CoordinateSystems = lcs_arr


