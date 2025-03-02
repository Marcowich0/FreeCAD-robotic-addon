"""FreeCAD init script of XXX module"""


# ***************************************************************************
# *   Copyright (c) 2015 John Doe john@doe.com                              *   
# *                                                                         *
# *   This file is part of the FreeCAD CAx development system.              *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENSE text file.                                 *
# *                                                                         *
# *   FreeCAD is distributed in the hope that it will be useful,            *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Lesser General Public License for more details.                   *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with FreeCAD; if not, write to the Free Software        *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
# *   USA                                                                   *
# *                                                                         *
# ***************************************************************************/

##FreeCAD.addImportType("My own format (*.own)", "importOwn")
##FreeCAD.addExportType("My own format (*.own)", "exportOwn")
print("I am executing some stuff here when FreeCAD starts!")

import sys
import os
import inspect

# Get the directory where the init file resides
current_dir = os.path.dirname(os.path.abspath(inspect.getsourcefile(lambda: 0)))

# Build the relative path to the Release folder
module_path = os.path.join(current_dir, "robot_dynamics_module", "build", "Release")

# Add the module path to sys.path
sys.path.append(module_path)