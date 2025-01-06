
import numpy as np
import pandas as pd


"""
------ASSUMPTIONS-----
- The shell is simply supported at its ends and the total compressive load is acting at its ends.
- Point masses
- No consideration of moments
"""

"""
point masses
name, mass, sandwich layer
"""

"""
outside of the sandwich layers:
solar panels, magnetometer, antenna, sun shield, engine, rcs thrusters
dimensions of the cylinder
"""

# Useful constants
launch_acceleration = 6*9.81  # m/s^2

# other geometric constants
core_radius = 0.42  # m
SC_minor_radius = 0.98  # m
SC_major_radius = 1.13  # m
SC_height = 2.5  # m

fuel_mass = 483.930 + 798.49  # kg
tanks_mass = 16.22 + 16.22  # kg


"""
compute the forces on each layer
verify the l-connectors for that layer
include the l-connectors mass


add all the layers to the central cylinder
check cylinder thickness/radius
get cylinder mass


"""


"""
wall 1 - magnetometer, 4rcs
wall 2 - solar panel
wall 3 - half of sunshield, 4rcs
wall 4 - nothing
wall 5 - half of sunshield, 4rcs
wall 6 - solar panel
"""


