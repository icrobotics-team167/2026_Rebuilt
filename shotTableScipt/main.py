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
from sleipnir.autodiff import VariableMatrix, atan2, hypot
from sleipnir.optimization import ExitStatus, Problem

# Physical characteristics
shooter_height = 20 * 0.0254  # m
min_pitch = np.deg2rad(45)  # rad
max_pitch = np.deg2rad(85)  # rad
g = 9.81  # m/s²
max_shooter_velocity = 14.5  # m/s
ball_mass = 0.5 / 2.205  # kg
ball_diameter = 5.91 * 0.0254  # m


# Solve settings
delta_pitch = np.deg2rad(2.5)
printResults = False


def lerp(a, b, t):
    return a + t * (b - a)


def f(x):
    """
    Apply the drag equation to a velocity.
    """
    # x' = x'
    # y' = y'
    # z' = z'
    # x" = −a_D(v_x)
    # y" = −a_D(v_y)
    # z" = −g − a_D(v_z)
    #
    # where a_D(v) = ½ρv² C_D A / m
    # (see https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation)
    rho = 1.204  # kg/m³
    C_D = 0.4
    m = ball_mass
    A = math.pi * ((ball_diameter / 2) ** 2)
    a_D = lambda v: 0.5 * rho * v**2 * C_D * A / m

    v_x = x[3, 0]
    v_y = x[4, 0]
    v_z = x[5, 0]
    return VariableMatrix(
        [[v_x], [v_y], [v_z], [-a_D(v_x)], [-a_D(v_y)], [-g - a_D(v_z)]]
    )


N = 40


def setup_problem(distance, target_height):
    """
    Set up the problem and any shared constraints between the two solve modes (min and fix vel)
    """
    # Robot initial state
    shooter_wrt_field = np.array([[0], [0], [shooter_height], [0.0], [0.0], [0.0]])

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

    # Dynamics constraints - RK4 integration
    h = dt
    for k in range(N - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]

        k1 = f(x_k)
        k2 = f(x_k + h / 2 * k1)
        k3 = f(x_k + h / 2 * k2)
        k4 = f(x_k + h * k3)
        problem.subject_to(x_k1 == x_k + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

    # Require final position is in center of target circle
    problem.subject_to(p[:, -1] == target_wrt_field[:3, :])

    # Require the final velocity is at least somewhat downwards by limiting horizontal velocity
    # and requiring negative vertical velocity
    problem.subject_to(v_z[-1] < -1)
    # Max horizontal velocity is 2.5 times the downwards velocity (~21 degrees from horizontal)
    problem.subject_to(hypot(v_x[-1], v_y[-1]) <= v_z[-1] * -5)

    problem.subject_to(
        atan2(v0_wrt_shooter[2, 0], hypot(v0_wrt_shooter[0, 0], (v0_wrt_shooter[1, 0])))
        >= min_pitch
    )

    return problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X


def min_velocity(distance, target_height):
    """
    Solve for minimum velocity.
    :returns: A tuple of [True, velocity, pitch, yaw, X] if it succeeds at a solve, and a tuple of[False, 0] if it fails.
    """
    problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X = setup_problem(
        distance, target_height
    )

    p_x = X[0, :]
    p_y = X[1, :]
    p_z = X[2, :]

    v = X[3:, :]

    # Position initial guess is linear interpolation between start and end position
    for k in range(N):
        p_x[k].set_value(lerp(shooter_wrt_field[0, 0], target_wrt_field[0, 0], k / N))
        p_y[k].set_value(lerp(shooter_wrt_field[1, 0], target_wrt_field[1, 0], k / N))
        p_z[k].set_value(lerp(shooter_wrt_field[2, 0], target_wrt_field[2, 0], k / N))

    # Velocity initial guess is max initial velocity toward target
    uvec_shooter_to_target = target_wrt_field[:3, :] - shooter_wrt_field[:3, :]
    uvec_shooter_to_target /= norm(uvec_shooter_to_target)
    for k in range(N):
        v[:, k].set_value(
            shooter_wrt_field[3:, :] + max_shooter_velocity * uvec_shooter_to_target
        )

    # Require initial velocity is less than max shooter velocity
    #
    #   √(v_x² + v_y² + v_z²) ≤ v
    #   v_x² + v_y² + v_z² ≤ v²
    #   vᵀv ≤ v²
    problem.subject_to(v0_wrt_shooter.T @ v0_wrt_shooter <= max_shooter_velocity**2)

    # Minimize initial velocity
    problem.minimize(v0_wrt_shooter.T @ v0_wrt_shooter)

    status = problem.solve()
    if status == ExitStatus.SUCCESS:
        # Initial velocity vector with respect to shooter
        v0 = v0_wrt_shooter.value()
        velocity = norm(v0)
        pitch = math.atan2(v0[2, 0], math.hypot(v0[0, 0], v0[1, 0]))
        time = T.value()

        if printResults:
            print(f"Min velocity solve:")
            print(f"Distance = {distance:.03f} m")
            print(f"Velocity = {velocity:.03f} m/s")
            print(f"Pitch = {np.rad2deg(pitch):.03f}°")
            print(f"Time = {time:.03f}s")

        return True, velocity, pitch, time, X
    print(f"Infeasible at distance {distance:.03f} m with status {status.name}")
    return False, 0


def fixed_pitch(distance, target_height, pitch, prev_X):
    """
    Solve for minimum velocity.
    :returns: A tuple of [True, velocity, pitch, yaw, X] if it succeeds at a solve, and a tuple of[False, 0] if it fails.
    """
    problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X = setup_problem(
        distance, target_height
    )

    prev_p_x = prev_X[0, :]
    prev_p_y = prev_X[1, :]
    prev_p_z = prev_X[2, :]

    prev_v = prev_X[3:, :]

    p_x = X[0, :]
    p_y = X[1, :]
    p_z = X[2, :]

    v = X[3:, :]

    # Position initial guess is last solve's position
    for k in range(N):
        p_x[k].set_value(prev_p_x[k].value())
        p_y[k].set_value(prev_p_y[k].value())
        p_z[k].set_value(prev_p_z[k].value())

    # Velocity initial guess is last solve's velocity
    for k in range(N):
        v[:, k].set_value(prev_v[:, k].value())

    problem.subject_to(
        atan2(v0_wrt_shooter[2, 0], hypot(v0_wrt_shooter[0, 0], (v0_wrt_shooter[1, 0])))
        == pitch
    )

    problem.minimize(T)

    status = problem.solve()
    if status == ExitStatus.SUCCESS:
        # Initial velocity vector with respect to shooter
        v0 = v0_wrt_shooter.value()
        velocity = norm(v0)
        pitch = math.atan2(v0[2, 0], math.hypot(v0[0, 0], v0[1, 0]))
        time = T.value()

        if printResults:
            print(f"Fixed pitch solve:")
            print(f"Distance = {distance:.03f} m")
            print(f"Velocity = {velocity:.03f} m/s")
            print(f"Pitch = {np.rad2deg(pitch):.03f}°")
            print(f"Time = {time:.03f}s")

        return True, velocity, pitch, time, X
    print(
        f"Infeasible at distance {distance:.03f} m and pitch {np.rad2deg(pitch)} with status {status.name}"
    )
    return False, 0


def max_velocity(distance, target_height, min_vel_solve):
    # Three stage solve: solve for the average of 90 degrees and the min vel solve's pitch,
    # then the average of that angle and 90 degrees, then do the max vel solve
    # The solver likes the fixed pitch solve more than it likes the max vel solve,
    # so use the fixed pitch solve to give the max vel solve a better initial guess.
    stage_1_solve = fixed_pitch(
        distance,
        target_height,
        (min_vel_solve[2] + np.deg2rad(90)) / 2,
        min_vel_solve[4],
    )
    if not stage_1_solve[0]:
        raise Exception("Fixed pitch solve stage 1 failed")
    stage_2_solve = fixed_pitch(
        distance,
        target_height,
        (stage_1_solve[2] + np.deg2rad(90)) / 2,
        stage_1_solve[4],
    )
    if not stage_2_solve[0]:
        raise Exception("Fixed pitch solve stage 2 failed")

    problem, shooter_wrt_field, target_wrt_field, v0_wrt_shooter, T, X = setup_problem(
        distance, target_height
    )

    fixed_pitch_X = stage_2_solve[4]

    fixed_pitch_p_x = fixed_pitch_X[0, :]
    fixed_pitch_p_y = fixed_pitch_X[1, :]
    fixed_pitch_p_z = fixed_pitch_X[2, :]

    fixed_pitch_v = fixed_pitch_X[3:, :]

    p_x = X[0, :]
    p_y = X[1, :]
    p_z = X[2, :]

    v = X[3:, :]
    v_x = X[3, :]
    v_y = X[4, :]
    v_z = X[5, :]

    # Position initial guess is the fixed pitch solve's position
    for k in range(N):
        p_x[k].set_value(fixed_pitch_p_x[k].value())
        p_y[k].set_value(fixed_pitch_p_y[k].value())
        p_z[k].set_value(fixed_pitch_p_z[k].value())

    # Velocity initial guess is the fixed pitch solve's velocity
    for k in range(N):
        v[:, k].set_value(fixed_pitch_v[:, k].value())

    # Require initial velocity is equal to max shooter velocity
    #
    #   √(v_x² + v_y² + v_z²) = v
    #   v_x² + v_y² + v_z² = v²
    #   vᵀv = v²
    problem.subject_to(v0_wrt_shooter.T @ v0_wrt_shooter == max_shooter_velocity**2)

    problem.minimize(T)

    status = problem.solve()
    if status == ExitStatus.SUCCESS:
        # Initial velocity vector with respect to shooter
        v0 = v0_wrt_shooter.value()
        velocity = norm(v0)
        pitch = math.atan2(v0[2, 0], math.hypot(v0[0, 0], v0[1, 0]))
        time = T.value()

        if printResults:
            print(f"Max velocity solve:")
            print(f"Distance = {distance:.03f} m")
            print(f"Velocity = {velocity:.03f} m/s")
            print(f"Pitch = {np.rad2deg(pitch):.03f}°")
            print(f"Time = {time:.03f}s")

        return True, velocity, pitch, time, X
    print(f"Infeasible at distance {distance:.03f} m with status {status.name}")
    return False, 0


def iterate_distance(file, distance, target_height):
    # Solve for minimum velocity
    min_vel_solve = min_velocity(distance, target_height)
    # If the position is possible, lerp between min velocity and max velocity
    # to search the in between velocities
    if min_vel_solve[0]:
        max_vel_solve = max_velocity(distance, target_height, min_vel_solve)
        if not max_vel_solve[0]:
            raise Exception("Max vel solve failed")
        stage_one_solve = fixed_pitch(
            distance,
            target_height,
            (max_pitch + min_vel_solve[2]) / 2,
            min_vel_solve[4],
        )
        max_pitch_solve = fixed_pitch(
            distance, target_height, max_pitch, stage_one_solve[4]
        )
        if not max_pitch_solve[0]:
            raise Exception("Max pitch solve failed")

        if max_pitch_solve[2] < max_vel_solve[2]:
            max_solve = max_pitch_solve
        else:
            max_solve = max_vel_solve

        min_max_pitch_delta = max_solve[2] - min_vel_solve[2]
        pitch_samples = math.ceil(min_max_pitch_delta / delta_pitch)

        file.write("    put(\n")
        file.write(f"        {distance},\n")
        file.write(
            f"        entry({min_vel_solve[1]}, new ShotResult({min_vel_solve[2]},"
            f" {min_vel_solve[3]})),\n"
        )
        prev_solve = min_vel_solve
        for i in range(1, pitch_samples - 1):
            pitch = lerp(min_vel_solve[2], max_solve[2], i / (pitch_samples - 1))
            solve = fixed_pitch(distance, target_height, pitch, prev_solve[4])
            if solve[0]:
                file.write(
                    f"        entry({solve[1]}, new ShotResult({solve[2]}, {solve[3]})),\n"
                )
                prev_solve = solve
            else:
                break
            if pitch + delta_pitch > max_solve[2]:
                break
        file.write(
            f"        entry({max_solve[1]}, new ShotResult({max_solve[2]}, "
            f"{max_solve[3]})));\n"
        )
        return min_vel_solve[2]


def write(
    file,
    target_height,
    min_distance,
    max_distance,
    distance_samples,
    distance_exponent,
    name,
):
    file.write("// Copyright (c) 2026 FRC 167\n")
    file.write("// https://github.com/icrobotics-team167\n")
    file.write("//\n")
    file.write("// Use of this source code is governed by an MIT-style\n")
    file.write("// license that can be found in the LICENSE file at\n")
    file.write("// the root directory of this project.\n\n")

    file.write("package frc.cotc.shooter;\n\n")

    file.write("import static java.util.Map.entry;\n\n")

    file.write(f"public final class {name} extends ShotMap " "{\n")
    file.write(f"  public {name}() " "{\n")

    for i in range(distance_samples):
        distance = lerp(
            min_distance,
            max_distance,
            (i / (distance_samples - 1)) ** distance_exponent,
        )
        iterate_distance(file, distance, target_height)

    file.write("  }\n")
    file.write("}\n")
    file.close()


if __name__ == "__main__":
    write(
        open("../src/main/java/frc/cotc/shooter/HubShotMap.java", "w"),
        72 * 0.0254,
        0.59,
        math.sqrt((8.062 / 2) ** 2 + ((158.1 + 47 / 2) * 0.0254) ** 2) + 6,
        20,
        2,
        "HubShotMap",
    )
    write(
        open("../src/main/java/frc/cotc/shooter/GroundShotMap.java", "w"),
        0,
        0.36,
        math.sqrt(8.062**2 + 16.541**2) - 1,
        15,
        2,
        "GroundShotMap",
    )
