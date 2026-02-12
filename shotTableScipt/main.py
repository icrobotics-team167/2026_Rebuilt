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
import sleipnir.autodiff as autodiff
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
    rho = 1.204  # kg/m³
    v = x[3:6, :]  # m/s
    v2 = (v.T @ v)[0, 0]
    C_D = 0.4
    r = ball_diameter / 2
    A = math.pi * r**2  # m²
    m = ball_mass
    F_D = 0.5 * rho * v2 * C_D * A

    v_mag = sqrt(v2)

    C_L = 0.00025

    v_hat = v / v_mag
    F_M = 0.5 * rho * C_L * A * v_mag * cross(v, omega)
    return block([[v], [-g - F_D / m * v_hat + F_M / m]])


N = 40


def solve(
    distance,
    target_height,
    vx,
    vy,
    last_x=None,
    last_shot_velocity=None,
    trying_again=False,
):
    """
    Solve for minimum velocity.
    :returns: A tuple of [True, velocity, pitch, yaw, X] if it succeeds at a solve, and a tuple of[False, 0] if it fails.
    """
    # Robot initial state
    shooter_wrt_field = np.array([[0], [0], [shooter_height], [vx], [vy], [0.0]])

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
    T.set_value(1)
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

    v_x = X[3, :]
    v_y = X[4, :]
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
    if last_shot_velocity is None:
        omega[0, 0].set_value(-max_shooter_velocity / ball_diameter)
    else:
        omega[0, 0].set_value(-last_shot_velocity / ball_diameter)

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

    target_radius = 47 * 0.0254 / 2
    wiggle_room = 0.05
    for k in range(N):
        dx = X[0, k] - target_wrt_field[0, 0]
        dy = X[1, k] - target_wrt_field[1, 0]
        dz = X[2, k] - (target_wrt_field[2, 0] + ball_diameter / 2 + wiggle_room)
        radius_outside = hypot(dx, dy) - (
            target_radius + ball_diameter / 2 + wiggle_room
        )
        problem.subject_to(autodiff.max(autodiff.max(radius_outside, dz), -v_z[k]) >= 0)

    # Require final position is in center of target circle
    problem.subject_to(p[:, -1] == target_wrt_field[:3, :])

    # Require the final velocity is at least somewhat downwards by limiting horizontal velocity
    # and requiring negative vertical velocity
    problem.subject_to(v_z[-1] < 0)

    p_x = X[0, :]
    p_y = X[1, :]
    p_z = X[2, :]

    v = X[3:, :]

    if last_x is None:
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
        X.set_value(last_x)

    # Require initial velocity is less than max shooter velocity
    #
    #   √(v_x² + v_y² + v_z²) ≤ v
    #   v_x² + v_y² + v_z² ≤ v²
    #   vᵀv ≤ v²
    problem.subject_to(v0_wrt_shooter.T @ v0_wrt_shooter <= max_shooter_velocity**2)
    problem.subject_to(v0_wrt_shooter.T @ v0_wrt_shooter >= 5)

    status = problem.solve(tolerance=1e-3, max_iterations=500)
    if printResults and status != ExitStatus.SUCCESS:
        print(f"Pre-solve 1 failed with status: {status.name}")

    pitch = atan2(
        v0_wrt_shooter[2, 0], hypot(v0_wrt_shooter[0, 0], (v0_wrt_shooter[1, 0]))
    )
    problem.subject_to(pitch <= max_pitch)
    problem.subject_to(pitch >= min_pitch)

    status = problem.solve(tolerance=1e-3, max_iterations=500)
    if printResults and status != ExitStatus.SUCCESS:
        print(f"Pre-solve 2 failed with status: {status.name}")

    # problem.minimize(v0_wrt_shooter.T @ v0_wrt_shooter)
    #
    # status = problem.solve(tolerance=1e-3,max_iterations=500)
    # if printResults and status != ExitStatus.SUCCESS:
    #     print(f"Pre-solve 3 failed with status: {status.name}")

    # Minimize time
    problem.minimize(T)

    status = problem.solve()
    if status == ExitStatus.SUCCESS:
        # Initial velocity vector with respect to shooter
        v0 = v0_wrt_shooter.value()
        velocity = norm(v0)
        pitch = math.atan2(v0[2, 0], math.hypot(v0[0, 0], v0[1, 0]))
        yaw = math.atan2(v0[1, 0], v0[0, 0])

        if printResults:
            print(
                f"Min velocity solve at distance {distance:.03f} m, vx = {vx:.03f} m/s, "
                f"vy = {vy:.03f} m/s"
            )
            print(f"Velocity = {velocity:.03f} m/s")
            print(f"Pitch = {np.rad2deg(pitch):.03f}°")
            print(f"Yaw = {np.rad2deg(yaw):.03f}°")

        return True, velocity, pitch, yaw, X.value()
    if not trying_again:
        if printResults:
            print(f"Solve failed with status {status.name}, trying again")
        return solve(distance, target_height, vx, vy, trying_again=True)
    print(
        f"Infeasible at distance {distance:.03f} m, vx = {vx:.03f} m/s, "
        f"vy = {vy:.03f} m/s with status {status.name}"
    )
    return False, 0


def iterate_distance(file, distance, target_height):
    file.write(f'    "{distance:.06f}": ' "{\n")
    file.write('      "map": {\n')
    angle_samples = 5
    velocity_samples = 5
    for i in range(angle_samples):
        angle = math.pi * i / (angle_samples - 1)
        file.write(f'        "{angle:.06f}": ' "{\n")
        file.write('          "map": {\n')
        last_x = None
        last_shot_velocity = None
        for j in range(velocity_samples):
            velocity = 6 * j / (velocity_samples - 1)
            status = solve(
                distance,
                target_height,
                velocity * math.cos(angle),
                velocity * math.sin(angle),
                last_x,
                last_shot_velocity,
            )
            if status[0]:
                shot_velocity, pitch, yaw, x = status[1:]
                file.write(f'            "{velocity}": ' "{\n")
                file.write(f'              "pitchRad": {pitch:.16f},\n')
                file.write('              "yaw": {\n')
                file.write(f'                "radians": {yaw:.16f}\n')
                file.write("              },\n")
                file.write(
                    f'              "velocityMetersPerSecond": {shot_velocity:.6f}\n'
                )
                file.write("            }")
                last_x = x
                last_shot_velocity = shot_velocity
            else:
                file.write(f'            "{velocity}": ' "{\n")
                file.write('              "pitchRad": -1,\n')
                file.write('              "yaw": {\n')
                file.write('                "radians": 0.0\n')
                file.write("              },\n")
                file.write('              "velocityMetersPerSecond": -1\n')
                file.write("            }")
                if j == 0 and i == 0:
                    print(
                        f"Warning: No valid stationary shot found at distance {distance:.03f} m"
                    )
            if j < velocity_samples - 1:
                file.write(",\n")
            else:
                file.write("\n")
        file.write("          }\n")
        file.write("        }")
        if i < angle_samples - 1:
            file.write(",\n")
        else:
            file.write("\n")
    file.write("      }\n")
    file.write("    }")


def write(
    target_height,
    min_distance,
    max_distance,
    distance_samples,
    name,
):
    file = open(f"../src/main/deploy/{name}.json", "w")

    file.write("{\n")
    file.write('  "map": {\n')

    for i in range(distance_samples):
        distance = lerp(
            min_distance,
            max_distance,
            i / (distance_samples - 1),
        )
        iterate_distance(file, distance, target_height)
        if i < distance_samples - 1:
            file.write(",\n")
        else:
            file.write("\n")

    file.write("  }\n")
    file.write("}\n")
    file.close()


if __name__ == "__main__":
    write(
        72 * 0.0254,
        1.2,
        math.hypot(8.069 / 2, (158.6 + 47 / 2) * 0.0254),
        30,
        "HubShotMap",
    )
    # write(0, 0.5, math.hypot(8.069, 16.541), 40, "GroundShotMap")
