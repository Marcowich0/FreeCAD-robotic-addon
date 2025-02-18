from main_utils import *
import itertools

def checkCollision():
    robot = get_robot()
    links = robot.Links
    intersect = False
    # Iterate over every unique pair of links
    for link_a, link_b in itertools.combinations(links, 2):
        # If necessary, use global placements; for example:
        # shape_a = link_a.Shape.copy()
        # shape_a.Placement = link_a.getGlobalPlacement()  
        # Otherwise, use the local shape:
        shape_a = link_a.Shape
        shape_b = link_b.Shape

        try:
            common_shape = shape_a.common(shape_b)
            if common_shape.Volume > 0:
                print(f"Collision between {link_a.Name} and {link_b.Name} is {common_shape.Volume}")
                intersect = True
        except Exception as e:
            pass
    return intersect