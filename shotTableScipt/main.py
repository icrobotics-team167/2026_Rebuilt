"""
FRC 2026 shooter trajectory optimization.

This program uses the Sleipnir NLP solver to find the initial pitch and yaw for a game
piece to hit the 2026 FRC game's target given an initial velocity.

This optimization problem formulation uses direct transcription of the flight dynamics, including
air resistance.

Based on the 2022 trajectory optimization example code by Tyler Veness.
https://github.com/SleipnirGroup/Sleipnir/blob/main/examples/frc_2022_shooter/main.py

Required packages:
- numpy
- sleipnirgroup-jormungandr
"""

import math

import numpy as np
from numpy.linalg import norm
from sleipnir.autodiff import VariableMatrix, atan2, block, cos, hypot, sin, sqrt
from sleipnir.optimization import ExitStatus, Problem

# Physical characteristics
shooter_height = 18 * 0.0254  # m
min_pitch = np.deg2rad(45)  # rad
max_pitch = np.deg2rad(85)  # rad
g = np.array([[0], [0], [9.81]])  # m/s²
max_shooter_velocity = 14.5  # m/s
ball_mass = 0.5 / 2.205  # kg
ball_diameter = 5.91 * 0.0254  # m


# Solve settings
printResults = False


def lerp(a, b, t):
    return a + t * (b - a)


def cross(u, v):
    return VariableMatrix(
        [
            [u[1, 0] * v[2, 0] - u[2, 0] * v[1, 0]],
            [-u[0, 0] * v[2, 0] + u[2, 0] * v[0, 0]],
            [u[0, 0] * v[1, 0] - u[1, 0] * v[0, 0]],
        ]
    )


def f(x, omega):
    # x' = x'
    # y' = y'
    # z' = z'
    # x" = −F_D(v)/m v̂_x
    # y" = −F_D(v)/m v̂_y
    # z" = −g − F_D(v)/m v̂_z
    #
    # Per https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation:
    #   F_D(v) = ½ρv²C_D A
    #   ρ is the fluid density in kg/m³
    #   v is the velocity magnitude in m/s
    #   C_D is the drag coefficient (dimensionless)
    #   A is the cross-sectional area of a circle in m²
    #   m is the mass in kg
    #   v̂ is the velocity direction unit vector
    rho = 1.221  # kg/m³
    v = x[3:6, :]  # m/s
    v2 = (v.T @ v)[0, 0]
    v_mag = sqrt(v2)
    r = ball_diameter / 2
    C_D = 0.4
    A = math.pi * r**2  # m²
    m = ball_mass
    F_D = 0.5 * rho * v2 * C_D * A

    C_L = 0.00025

    v_hat = v / v_mag
    F_M = 0.5 * rho * C_L * A * v_mag * cross(v, omega)
    return block([[v], [-g - F_D / m * v_hat + F_M / m]])


N = 40


def solve(
    distance,
    target_height,
    last_solve=None,
    mode=0,
    target_velocity=None,
    trying_again=False,
):
    """
    Solve for minimum velocity.
    :returns: A tuple of [True, velocity, pitch, yaw, X] if it succeeds at a solve, and a tuple of[False, 0] if it fails.
    """
    # Robot initial state
    shooter_wrt_field = np.array(
        [
            [0],
            [0],
            [shooter_height],
            [0],
            [0],
            [0],
        ]
    )

    target_wrt_field = np.array(
        [
            [distance],
            [0],
            [target_height],
            [0.0],
            [0.0],
            [0.0],
        ]
    )

    problem = Problem()

    # Set up duration decision variables
    T = problem.decision_variable()
    problem.subject_to(T >= 0)
    if last_solve is None:
        T.set_value(1)
    else:
        T.set_value(last_solve[3])
    dt = T / N

    # Ball state in field frame
    #
    #     [x position]
    #     [y position]
    #     [z position]
    # x = [x velocity]
    #     [y velocity]
    #     [z velocity]
    X = problem.decision_variable(6, N)

    p = X[:3, :]

    v_z = X[5, :]

    v0_wrt_shooter = X[3:, :1] - shooter_wrt_field[3:, :]

    # Shooter initial position
    problem.subject_to(p[:, :1] == shooter_wrt_field[:3, :])

    omega_magnitude = v0_wrt_shooter.T @ v0_wrt_shooter / ball_diameter

    omega = problem.decision_variable(3, 1)
    problem.subject_to(
        omega[0, 0]
        == omega_magnitude
        * cos(atan2(v0_wrt_shooter[1, 0], v0_wrt_shooter[0, 0]) - math.pi / 2)
    )
    problem.subject_to(
        omega[1, 0]
        == omega_magnitude
        * sin(atan2(v0_wrt_shooter[1, 0], v0_wrt_shooter[0, 0]) - math.pi / 2)
    )
    problem.subject_to(omega[2, 0] == 0)
    if last_solve is None:
        omega[0, 0].set_value(-max_shooter_velocity / ball_diameter)
    else:
        omega[0, 0].set_value(-last_solve[1] / ball_diameter)

    # Dynamics constraints - RK4 integration
    h = dt
    for k in range(N - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]

        k1 = f(x_k, omega)
        k2 = f(x_k + h / 2 * k1, omega)
        k3 = f(x_k + h / 2 * k2, omega)
        k4 = f(x_k + h * k3, omega)
        problem.subject_to(x_k1 == x_k + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

    # Require final position is in center of target circle
    problem.subject_to(p[:, -1] == target_wrt_field[:3, :])

    # Require the final velocity is at least somewhat downwards by limiting horizontal velocity
    # and requiring negative vertical velocity
    problem.subject_to(v_z[-1] < -1)

    p_x = X[0, :]
    p_y = X[1, :]
    p_z = X[2, :]

    v = X[3:, :]

    if last_solve is None:
        # Position initial guess is linear interpolation between start and end position
        for k in range(N):
            p_x[k].set_value(
                lerp(shooter_wrt_field[0, 0], target_wrt_field[0, 0], k / N)
            )
            p_y[k].set_value(
                lerp(shooter_wrt_field[1, 0], target_wrt_field[1, 0], k / N)
            )
            p_z[k].set_value(
                lerp(shooter_wrt_field[2, 0], target_wrt_field[2, 0], k / N)
            )

        # Velocity initial guess is max initial velocity toward target
        uvec_shooter_to_target = target_wrt_field[:3, :] - shooter_wrt_field[:3, :]
        uvec_shooter_to_target /= norm(uvec_shooter_to_target)
        for k in range(N):
            v[:, k].set_value(
                shooter_wrt_field[3:, :] + max_shooter_velocity * uvec_shooter_to_target
            )
    else:
        X.set_value(last_solve[4])

    #   √(v_x² + v_y² + v_z²) ≤ v
    #   v_x² + v_y² + v_z² ≤ v²
    #   vᵀv ≤ v²
    initial_velocity_squared = v0_wrt_shooter.T @ v0_wrt_shooter
    # Require initial velocity is less than max shooter velocity
    problem.subject_to(initial_velocity_squared <= max_shooter_velocity**2)

    pitch = atan2(
        v0_wrt_shooter[2, 0], hypot(v0_wrt_shooter[0, 0], (v0_wrt_shooter[1, 0]))
    )
    problem.subject_to(pitch <= max_pitch)
    problem.subject_to(pitch >= min_pitch)
    if last_solve is not None:
        problem.subject_to(pitch > last_solve[2])

    if mode == 0:
        # Minimize initial velocity
        problem.minimize(initial_velocity_squared)
    elif mode == 1:
        if last_solve is not None:
            problem.subject_to(initial_velocity_squared > last_solve[1] ** 2)
        # Maximize initial velocity
        problem.maximize(initial_velocity_squared)
    elif mode == 2:
        problem.subject_to(v0_wrt_shooter.T @ v0_wrt_shooter == target_velocity**2)

    status = problem.solve()
    if status == ExitStatus.SUCCESS:
        # Initial velocity vector with respect to shooter
        v0 = v0_wrt_shooter.value()
        velocity = norm(v0)
        pitch = pitch.value()
        time = T.value()

        if printResults:
            print(f"Mode {mode} solve at distance {distance:.03f} m")
            print(f"Velocity = {velocity:.03f} m/s")
            print(f"Pitch = {np.rad2deg(pitch):.03f}°")
            print(f"Time = {time:.03f} s")

        return True, velocity, pitch, time, X.value()
    if not trying_again:
        if printResults:
            print(f"Solve failed with status {status.name}, trying again")
        return solve(
            distance,
            target_height,
            mode=mode,
            target_velocity=target_velocity,
            trying_again=True,
        )
    if mode != 2:
        print(f"Mode {mode} solve failed at distance {distance:.03f} m")
    return False, 0


def iterate_distance(file, distance, target_height, last_min_vel_solve):
    min_vel_solve = solve(distance, target_height, last_min_vel_solve)
    if min_vel_solve[0]:
        if last_min_vel_solve is None:
            file.write("\n")
        else:
            file.write(",\n")
        file.write(f'    "{distance:.06f}": ' "{\n")
        file.write('      "map": {\n')
        iterate_shot_velocity(file, distance, target_height, min_vel_solve)
        file.write("      }\n")
        file.write("    }")
        return min_vel_solve
    else:
        return None


def iterate_shot_velocity(file, distance, target_height, min_vel_solve):
    solves = [min_vel_solve[1:4]]
    base_delta_vel = 0.5
    delta_vel = base_delta_vel
    if min_vel_solve[1] < max_shooter_velocity:
        target_velocity = min_vel_solve[1] + delta_vel
        last_solve = min_vel_solve
        while target_velocity < max_shooter_velocity:
            fixed_vel_solve = solve(
                distance,
                target_height,
                last_solve,
                mode=2,
                target_velocity=target_velocity,
            )
            if fixed_vel_solve[0]:
                solves.append(fixed_vel_solve[1:4])
                last_solve = fixed_vel_solve
                delta_vel = base_delta_vel
                target_velocity += delta_vel
            else:
                if delta_vel < 0.01:
                    break
                target_velocity -= delta_vel
                delta_vel /= 2
                target_velocity += delta_vel

        max_vel_solve = solve(
            distance,
            target_height,
            last_solve,
            mode=1,
        )
        if max_vel_solve[0] and max_vel_solve[0] > solves[-1][0]:
            solves.append(max_vel_solve[1:4])
    for i in range(len(solves)):
        shot_velocity, pitch, time = solves[i]
        file.write(f'        "{shot_velocity}": ' "{\n")
        file.write(f'          "pitchRad": {pitch:.16f},\n')
        file.write(f'          "timeOfFlightSeconds": {time}\n')
        file.write("        }")
        if i < len(solves) - 1:
            file.write(",\n")
        else:
            file.write("\n")


def write(
    target_height,
    min_distance,
    max_distance,
    base_delta_distance,
    name,
):
    file = open(f"../src/main/deploy/{name}.json", "w")

    file.write("{\n")
    file.write('  "map": {')

    last_min_vel_solve = None
    delta_distance = base_delta_distance
    distance = min_distance
    while True:
        distance_solve = iterate_distance(
            file, distance, target_height, last_min_vel_solve
        )
        if distance_solve is not None:
            last_min_vel_solve = distance_solve
            delta_distance = base_delta_distance
            distance += delta_distance
            if distance > max_distance:
                file.write("\n")
                break
        else:
            distance -= delta_distance
            delta_distance /= 2
            if delta_distance < 0.01:
                delta_distance = base_delta_distance
            distance += delta_distance * 2
            if distance > max_distance:
                file.write("\n")
                break

    file.write("  }\n")
    file.write("}\n")
    file.close()
    print(f"Done writing {name}.json")


if __name__ == "__main__":
    write(
        72 * 0.0254,
        0.3,
        10,
        0.5,
        "HubShotMap",
    )
    write(0, 0.75, math.hypot(8.069, 16.541), .5, "GroundShotMap")
