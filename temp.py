import sys
import os

module_path = os.path.join(os.path.dirname(__file__), "robot_dynamics_module", "build", "Release")
if module_path not in sys.path:
    sys.path.insert(0, module_path)
    
import inverse_kinematics_cpp

print("Testing if the module is imported correctly")
print(inverse_kinematics_cpp)