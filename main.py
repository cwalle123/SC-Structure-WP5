

import math as m
import numpy as np
# from dataclasses import dataclass

from inputs import *


# determine density from the material
material_properties = {
    'Al 7075 T6': (71.7e9, 430e6, .5 * 430e6, 2810, 2.36e-5),
    'Ti-6Al-4V': (113.8e9, 880e6, .5 * 880e6, 4430, 8.6e-6),
    '18-8 Stainless Steel': (193e9, 205e6, .5 * 205e6, 7900, 16.5e-6)
}

# decide on the material
elastic_modulus, sigma_yield, tau_yield, density, thermal_expansion = material_properties["Al 7075 T6"]

# determine mass of sandwich layer
sandwich_area_density = 1.999  # kg/m^2
sandwich_area = 3 * np.sqrt(3) / 2 * SC_major_radius**2 - np.pi * core_radius**2
sandwich_layer_mass = sandwich_area * sandwich_area_density  # kg


def get_lugs(load, multiple):
    number = np.ceil((load / 408) / multiple) * multiple
    mass = number * 0.00235  # kg
    return mass, number


def euler_buckling(L, R, E, F_z, FoS, dt=0.0001):
    t = 0  # Initial thickness
    sigma_applied = np.inf  # Arbitrary large value to start the loop
    sigma_critical = 0

    while FoS * sigma_applied > sigma_critical and sigma_applied > sigma_yield:
        t += dt
        # Cross-sectional properties
        moment_inertia = np.pi * t * R ** 3  # Moment of inertia for a thin-walled cylinder
        cross_area = 2 * np.pi * R * t  # Cross-sectional area of the shell

        # Critical and applied stresses
        sigma_critical = (np.pi**2 * E * moment_inertia) / (cross_area * L**2)
        sigma_applied = F_z / cross_area

    return t, cross_area, moment_inertia, sigma_applied, sigma_critical


def shell_buckling(radius_shell, length_shell, E_modulus, pressure_difference, F_z, FoS, poisson=0.33, dt=0.0001):
    thickness_shell = 0  # Initial thickness
    applied_stress = np.inf  # Ensure loop starts
    critical_stress = 0  # Ensure loop starts

    while FoS * applied_stress > critical_stress:
        thickness_shell += dt
        # cross area
        cross_area = 2 * np.pi * radius_shell * thickness_shell  # Cross-sectional area of the shell

        # some weird value called Q
        Q = pressure_difference / E_modulus * (radius_shell / thickness_shell) ** 2
        # we need lambda for k
        lamda = np.sqrt(12 / (np.pi ** 4) * length_shell**4 / (radius_shell * thickness_shell)**2 * (1 - poisson**2))
        # another thingy named k
        k = lamda + 12 / (np.pi ** 4) * length_shell**4 / (radius_shell * thickness_shell)**2 * (1 - poisson**2) * 1/lamda
        # critical stress for shell buckling
        critical_stress = (1.983 - 0.983 * np.exp(-23.14 * Q)) * k * np.pi**2 * E_modulus / (12 * (1 - poisson**2)) * (
                    thickness_shell / length_shell) ** 2
        # applied stress
        applied_stress = F_z / cross_area

    return cross_area, thickness_shell, applied_stress, critical_stress


class Wall:
    def __init__(self, wall_number, thickness, component_mass):
        self.number = wall_number
        self.own_mass = thickness * 2.5 * 1.13 * density  # for thickness in meters
        last_mass = self.own_mass + component_mass

        running = True
        while running:
            load = last_mass * launch_acceleration

            lug_m, lug_n = get_lugs(load, 4)

            mass = last_mass + lug_m
            if abs(last_mass - mass) > 0.001:
                running = False
            last_mass = mass

        self.lug_number = lug_n
        self.mass = last_mass
        print(f"Wall {self.number} has a mass {self.mass:.3g} kg.")


class Layer:
    def __init__(self, layer_number, component_mass, effective_wall_mass):
        self.number = layer_number
        last_mass = sandwich_layer_mass + component_mass + effective_wall_mass

        running = True
        while running:
            load = last_mass * launch_acceleration

            lug_m, lug_n = get_lugs(load, 2)

            mass = last_mass + lug_m
            if abs(last_mass - mass) > 0.001:
                running = False
            last_mass = mass

        self.lug_number = lug_n
        self.mass = last_mass
        print(f"Layer {self.number} bears a mass of {self.mass:.3g} kg.")


def main():
    wall_thickness = 0.002
    wall1 = Wall(1, wall_thickness, 1.4+5.2+0.375+5)
    wall2 = Wall(2, wall_thickness, 10.785+5)
    wall3 = Wall(3, wall_thickness, 19/3+1.4+5)
    wall4 = Wall(4, wall_thickness, 19/3+5)
    wall5 = Wall(5, wall_thickness, 19/3+1.4+5)
    wall6 = Wall(6, wall_thickness, 10.785+5)
    walls = wall1, wall2, wall3, wall4, wall5, wall6

    print()
    total_wall_mass = sum(wall.mass for wall in walls)
    wall_mass_contribution = total_wall_mass / 4

    layer1 = Layer(1, 73.5 , wall_mass_contribution)
    layer2 = Layer(2, 17.67, wall_mass_contribution)
    layer3 = Layer(3, 14.16, wall_mass_contribution)
    layer4 = Layer(4, 35.6 , wall_mass_contribution)

    print()
    layers = layer1, layer2, layer3, layer4
    mass_except_core = sum(layer.mass for layer in layers) + fuel_mass + tanks_mass
    load_except_core = mass_except_core * launch_acceleration

    safety_factor = 1.5

    t, cross_area, moment_inertia, sigma_applied, sigma_critical = euler_buckling(SC_height, core_radius, elastic_modulus, load_except_core, safety_factor)
    print(f"Euler buckling critical stress is {sigma_critical:.3g}, applied is {sigma_applied:.3g}.")
    print(f"The mass of the core is now {cross_area * SC_height * density:.3g} kg")
    print(f"Thickness: {t}, Safety Margin {sigma_critical/sigma_applied - 1}")
    print()

    p_difference = 15e0
    crosssection_area, thickness_shell, applied_stress, critical_stress = shell_buckling(core_radius, SC_height, elastic_modulus, p_difference, load_except_core, safety_factor)
    print(f"Shell buckling critical stress is {critical_stress:.3g}, applied is {applied_stress:.3g}.")
    print(f"The mass of the core is now {crosssection_area * SC_height * density:.3g} kg")
    print(f"Thickness: {thickness_shell}, Safety Margin {critical_stress/applied_stress - 1}")
    print()

    SC_total_mass = mass_except_core + max(cross_area * SC_height * density, crosssection_area * SC_height * density)
    print(f"Spacecraft loaded mass is {SC_total_mass:.0f} kg")

if __name__ == "__main__":
    main()
