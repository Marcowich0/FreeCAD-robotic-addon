import FreeCAD
import FreeCADGui


def get_robot():
    for obj in FreeCAD.ActiveDocument.Objects:
        if hasattr(obj, 'Type') and obj.Type == 'Robot':
            return obj
    return None




def correctLinkPosition():
    doc = FreeCAD.ActiveDocument
    robot = get_robot()
    for i, body in enumerate(robot.Bodies):
        
        o_A_dh = doc.getObject(f"LCS_link_{i}").Placement.Matrix
        b_A_c = doc.getObject(f"Angle_reference_LCS_{i}").Placement.Matrix

        body.Placement.Matrix = o_A_dh * b_A_c.inverse()
