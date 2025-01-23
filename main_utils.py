import FreeCAD
import FreeCADGui


def get_robot():
    for obj in FreeCAD.ActiveDocument.Objects:
        if hasattr(obj, 'Type') and obj.Type == 'Robot':
            return obj
    return None