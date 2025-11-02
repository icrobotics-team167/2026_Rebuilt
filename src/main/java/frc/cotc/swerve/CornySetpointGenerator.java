// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.cotc.Robot;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

public class CornySetpointGenerator {
  private final SwerveDriveKinematics kinematics;
  private final Translation2d[] modPosReciprocals = new Translation2d[4];
  private final SimpleMatrix moduleForceIK = new SimpleMatrix(4, 3);

  private final double wheelRadiusMeters;
  private final DCMotor driveMotor;
  private final double statorCurrentLimitAmps;
  private final double supplyCurrentLimitAmps;
  public final double maxLinearSpeedMetersPerSec;
  public final double maxAngularSpeedRadPerSec;
  private final double robotMassKg;
  private final double robotMOIKgMetersSquared;
  private final double maxDeltaThetaRad;
  private final double maxDeltaVelMps;

  // Only enable internal state logging in sim/replay to free up CPU/RAM/disk on the RIO
  private final boolean enableLogging = Robot.isSimulation();

  CornySetpointGenerator(
      Translation2d[] moduleLocations,
      double wheelRadiusMeters,
      DCMotor driveMotor,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double maxSteerSpeedRadPerSec,
      double robotMassKg,
      double robotMOIKgMetersSquared,
      SwerveModuleState[] initialStates) {
    kinematics = new SwerveDriveKinematics(moduleLocations);
    for (int i = 0; i < 4; i++) {
      modPosReciprocals[i] =
          new Translation2d(1.0 / moduleLocations[i].getNorm(), moduleLocations[i].getAngle());
    }
    this.wheelRadiusMeters = wheelRadiusMeters;
    this.driveMotor = driveMotor;
    this.statorCurrentLimitAmps = statorCurrentLimitAmps;
    this.supplyCurrentLimitAmps = supplyCurrentLimitAmps;
    maxLinearSpeedMetersPerSec = driveMotor.freeSpeedRadPerSec * wheelRadiusMeters;
    maxAngularSpeedRadPerSec = maxLinearSpeedMetersPerSec / moduleLocations[0].getNorm();
    maxDeltaThetaRad = maxSteerSpeedRadPerSec * Robot.defaultPeriodSecs;
    this.robotMassKg = robotMassKg;
    this.robotMOIKgMetersSquared = robotMOIKgMetersSquared;
    maxDeltaVelMps =
        (4 * driveMotor.KtNMPerAmp * statorCurrentLimitAmps / wheelRadiusMeters)
            / robotMassKg
            * Robot.defaultPeriodSecs;
    Logger.recordOutput(
        "Swerve/Drive Calculations/Traction/Max acceleration mpss",
        maxDeltaVelMps / Robot.defaultPeriodSecs);
    lastStates = initialStates;
  }

  private SwerveModuleState[] lastStates;

  Setpoint calculateNextState(ChassisSpeeds commandedChassisSpeeds) {
    var desiredStates = kinematics.toSwerveModuleStates(commandedChassisSpeeds);
    // Make sure desiredState respects velocity limits.
    // Desaturate
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxLinearSpeedMetersPerSec);
    commandedChassisSpeeds = kinematics.toChassisSpeeds(desiredStates);
    // Discretize
    commandedChassisSpeeds =
        ChassisSpeeds.discretize(commandedChassisSpeeds, Robot.defaultPeriodSecs);
    desiredStates = kinematics.toSwerveModuleStates(commandedChassisSpeeds);
    // Desaturate again
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxLinearSpeedMetersPerSec);
    commandedChassisSpeeds = kinematics.toChassisSpeeds(desiredStates);

    Logger.recordOutput("Swerve/Commanded Chassis Speeds", commandedChassisSpeeds);
    Logger.recordOutput("Swerve/Drive Calculations/Last Module Speeds", lastStates);
    Logger.recordOutput("Swerve/Drive Calculations/Desired Module Speeds", desiredStates);

    var lastChassisSpeeds = kinematics.toChassisSpeeds(lastStates);

    // Apply optimizations
    {
      // If the desired speed is to stop, be lazy and not move
      for (int i = 0; i < 4; i++) {
        if (MathUtil.isNear(0, desiredStates[i].speedMetersPerSecond, .01)) {
          desiredStates[i].angle = lastStates[i].angle;
        }
      }

      // Calculate if it's less motion to stop first, move smoothly, or move smoothly but with
      // the desired state flipped
      boolean allModulesShouldStop = true;
      for (int i = 0; i < 4; i++) {
        var deltaVelToStop =
            Math.abs(lastStates[i].speedMetersPerSecond)
                + Math.abs(desiredStates[i].speedMetersPerSecond);
        double deltaVelToSmooth = 0;
        for (int j = 0; j < 10; j++) {
          double lastX =
              MathUtil.interpolate(
                  lastStates[i].speedMetersPerSecond,
                  desiredStates[i].speedMetersPerSecond,
                  (j) / 10.0);
          double newX =
              MathUtil.interpolate(
                  lastStates[i].speedMetersPerSecond,
                  desiredStates[i].speedMetersPerSecond,
                  (j + 1) / 10.0);
          var lastTheta = lastStates[i].angle.interpolate(desiredStates[i].angle, (j) / 10.0);
          var newTheta = lastStates[i].angle.interpolate(desiredStates[i].angle, (j + 1) / 10.0);
          deltaVelToSmooth +=
              Math.hypot(
                  newX - lastX,
                  newTheta.minus(lastTheta).getRadians()
                      * MathUtil.interpolate(
                          lastStates[i].speedMetersPerSecond,
                          desiredStates[i].speedMetersPerSecond,
                          j / 10.0));
        }
        double deltaVelToSmoothFlipped = 0;
        for (int j = 0; j < 10; j++) {
          double lastX =
              MathUtil.interpolate(
                  lastStates[i].speedMetersPerSecond,
                  -desiredStates[i].speedMetersPerSecond,
                  (j) / 10.0);
          double newX =
              MathUtil.interpolate(
                  lastStates[i].speedMetersPerSecond,
                  -desiredStates[i].speedMetersPerSecond,
                  (j + 1) / 10.0);
          var lastTheta =
              lastStates[i].angle.interpolate(
                  desiredStates[i].angle.plus(Rotation2d.kPi), (j) / 10.0);
          var newTheta =
              lastStates[i].angle.interpolate(
                  desiredStates[i].angle.plus(Rotation2d.kPi), (j + 1) / 10.0);
          deltaVelToSmoothFlipped +=
              Math.hypot(
                  newX - lastX,
                  newTheta.minus(lastTheta).getRadians()
                      * MathUtil.interpolate(
                          lastStates[i].speedMetersPerSecond,
                          -desiredStates[i].speedMetersPerSecond,
                          j / 10.0));
        }

        if (enableLogging) {
          Logger.recordOutput(
              "Swerve/Drive Calculations/Optimizations/" + i + "/Delta Vels To Stop",
              deltaVelToStop);
          Logger.recordOutput(
              "Swerve/Drive Calculations/Optimizations/" + i + "/Delta Vels To Smooth",
              deltaVelToSmooth);
          Logger.recordOutput(
              "Swerve/Drive Calculations/Optimizations/" + i + "/Delta Vels To Smooth Flipped",
              deltaVelToSmoothFlipped);
        }

        // If it's faster to move smoothly, we shouldn't stop first
        if (deltaVelToSmooth < deltaVelToStop + .01
            || deltaVelToSmoothFlipped < deltaVelToStop + .01) {
          allModulesShouldStop = false;
          // If it's faster to flip the direction when moving smoothly, do so
          if (deltaVelToSmoothFlipped < deltaVelToSmooth) {
            desiredStates[i].speedMetersPerSecond *= -1;
            desiredStates[i].angle = desiredStates[i].angle.plus(Rotation2d.kPi);
            if (enableLogging) {
              Logger.recordOutput(
                  "Swerve/Drive Calculations/Optimizations/" + i + "/Faster To:", "Smooth Flipped");
            }
          } else if (enableLogging) {
            Logger.recordOutput(
                "Swerve/Drive Calculations/Optimizations/" + i + "/Faster To:", "Smooth");
          }
        } else if (enableLogging) {
          Logger.recordOutput(
              "Swerve/Drive Calculations/Optimizations/" + i + "/Faster To:", "Stop");
        }
      }
      // If it's faster to stop for all modules, then do so
      // Only stopping when it's faster for all modules avoids scenarios where one module is
      // moving to stop but others aren't, which is slower/kinematically infeasible
      if (allModulesShouldStop) {
        for (int i = 0; i < 4; i++) {
          desiredStates[i].speedMetersPerSecond = 0;
        }
      }
      if (enableLogging) {
        Logger.recordOutput(
            "Swerve/Drive Calculations/Optimizations/All Modules Should Stop",
            allModulesShouldStop);
      }
    }

    // Limit max steer speed
    // Technically this should use a motion profile instead of a linear steer speed as the
    // motors can't accelerate infinitely fast, but steer motors have hilariously overkill
    // torques compared to the system inertia so acceleration doesn't really matter
    {
      double t = 1;
      double[] desiredDeltaThetasRad = new double[4];
      for (int i = 0; i < 4; i++) {
        desiredDeltaThetasRad[i] = desiredStates[i].angle.minus(lastStates[i].angle).getRadians();
        // If the magnitude of the desired change in angle is greater than the max allowed steer
        // speed, then limit t based on that
        if (Math.abs(desiredDeltaThetasRad[i]) > maxDeltaThetaRad) {
          t = Math.min(t, maxDeltaThetaRad / Math.abs(desiredDeltaThetasRad[i]));
        }
      }
      if (enableLogging) {
        Logger.recordOutput(
            "Swerve/Drive Calculations/Steer Speed/Desired Delta Thetas Rad",
            desiredDeltaThetasRad);
        Logger.recordOutput("Swerve/Drive Calculations/Steer Speed/t", t);
      }

      updateDesiredStates(desiredStates, t);
    }

    // Limit acceleration based on max traction
    // This includes sideways acceleration as well in order to minimize wheel wear from slip
    {
      double t = 1;
      double[] desiredDeltaVels = new double[4];
      // Loop over all the modules and see if they violate the max acceleration constraint
      // Technically this math isn't fully accurate but it does get closer to the true value as
      // ∆t -> 0, and with a ∆t = 0.02 seconds, it's certified Good Enough™️
      for (int i = 0; i < 4; i++) {
        desiredDeltaVels[i] =
            Math.hypot(
                desiredStates[i].speedMetersPerSecond - lastStates[i].speedMetersPerSecond,
                desiredStates[i].angle.minus(lastStates[i].angle).getRadians()
                    * lastStates[i].speedMetersPerSecond);
        if (desiredDeltaVels[i] > maxDeltaVelMps) {
          t = Math.min(t, maxDeltaVelMps / desiredDeltaVels[i]);
        }
      }

      updateDesiredStates(desiredStates, t);

      if (enableLogging) {
        Logger.recordOutput(
            "Swerve/Drive Calculations/Traction/Desired Delta Vels mps", desiredDeltaVels);
        Logger.recordOutput("Swerve/Drive Calculations/Traction/t", t);
        double[] finalDeltaVels = new double[4];
        for (int i = 0; i < 4; i++) {
          finalDeltaVels[i] =
              Math.hypot(
                  desiredStates[i].speedMetersPerSecond - lastStates[i].speedMetersPerSecond,
                  desiredStates[i].angle.minus(lastStates[i].angle).getRadians()
                      * lastStates[i].speedMetersPerSecond);
        }
        Logger.recordOutput(
            "Swerve/Drive Calculations/Traction/Final Delta Vels mps", finalDeltaVels);
      }
    }

    // Limit drive motor dynamics
    // This prevents infeasible velocities/accelerations being commanded
    {
      // Create the inverse kinematics matrix for the module forces
      for (int i = 0; i < 4; i++) {
        moduleForceIK.setRow(
            i,
            0,
            lastStates[i].angle.getCos(),
            lastStates[i].angle.getSin(),
            (-modPosReciprocals[i].getY() * lastStates[i].angle.getCos()
                + modPosReciprocals[i].getX() * lastStates[i].angle.getSin()));
      }

      // Calculate desired module forces
      var desiredModuleForces =
          getModuleForces(lastChassisSpeeds, kinematics.toChassisSpeeds(desiredStates));
      if (enableLogging) {
        var desiredModuleForceVisualizations = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
          desiredModuleForceVisualizations[i] =
              new SwerveModuleState(desiredModuleForces.get(i, 0), lastStates[i].angle);
        }
        Logger.recordOutput(
            "Swerve/Drive Calculations/Motor Dynamics/Desired Module Forces",
            desiredModuleForceVisualizations);
      }

      // Calculate max module forces
      var lastWheelVelsRadPerSec = new double[4];
      var maxStatorCurrentsAmps = new double[4];
      var maxModuleForceMagnitudesNewtons = new double[4];
      for (int i = 0; i < 4; i++) {
        // Convert m/s to rad/s
        double lastWheelVelRadPerSec = lastStates[i].speedMetersPerSecond / wheelRadiusMeters;
        lastWheelVelsRadPerSec[i] = lastWheelVelRadPerSec;

        // Calculate max stator current given the current velocity and battery voltage
        var maxStatorCurrentAmps =
            desiredModuleForces.get(i, 0) >= 0
                ? getMaxTorqueCurrent(lastWheelVelRadPerSec)
                : -getMaxTorqueCurrent(-lastWheelVelRadPerSec);
        maxStatorCurrentsAmps[i] = maxStatorCurrentAmps;
        // Calculate max module force given the max stator current
        maxModuleForceMagnitudesNewtons[i] =
            maxStatorCurrentAmps * driveMotor.KtNMPerAmp / wheelRadiusMeters;
      }

      if (enableLogging) {
        var maxModuleForceVisualizations = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
          maxModuleForceVisualizations[i] =
              new SwerveModuleState(maxModuleForceMagnitudesNewtons[i], lastStates[i].angle);
        }
        Logger.recordOutput(
            "Swerve/Drive Calculations/Motor Dynamics/Last Wheel Vel Rad Per Sec",
            lastWheelVelsRadPerSec);
        Logger.recordOutput(
            "Swerve/Drive Calculations/Motor Dynamics/Max Stator Current Amps",
            maxStatorCurrentsAmps);
        Logger.recordOutput(
            "Swerve/Drive Calculations/Motor Dynamics/Max Module Forces",
            maxModuleForceVisualizations);
      }

      double largestMagnitude = 0;
      for (int i = 0; i < 4; i++) {
        largestMagnitude = Math.max(largestMagnitude, Math.abs(maxModuleForceMagnitudesNewtons[i]));
      }
      boolean exceededLimit = false;
      // Loop over all the modules and see if they violate the max force constraints
      for (int i = 0; i < 4; i++) {
        // Small max force constraints (<15% of the largest max force constraint) are ignored
        // as they are relatively negligible and cause issues with slow transitions between
        // states
        // Some amount of kinematic infeasibility is worth it to avoid control weirdness
        if (Math.abs(maxModuleForceMagnitudesNewtons[i]) / largestMagnitude > .15) {
          exceededLimit = desiredModuleForces.get(i, 0) / maxModuleForceMagnitudesNewtons[i] > 1.01;
          if (exceededLimit) {
            if (enableLogging) {
              Logger.recordOutput(
                  "Swerve/Drive Calculations/Motor Dynamics/Module Exceeding Limits", i);
            }
            break;
          }
        }
      }

      if (enableLogging) {
        if (!exceededLimit) {
          Logger.recordOutput(
              "Swerve/Drive Calculations/Motor Dynamics/Module Exceeding Limits", -1);
        }
        Logger.recordOutput(
            "Swerve/Drive Calculations/Motor Dynamics/Largest Magnitude", largestMagnitude);
      }

      // If there is a max force constraint violation, binary search to find the largest lerp
      // value t that doesn't violate any constraints
      if (exceededLimit) {
        double high_t = 1;
        double low_t = 0;
        double t;
        int iterationLimit = 10;
        int iterations = 0;
        while (true) {
          var avg_t = (high_t + low_t) / 2;
          // Iteration limit to prevent it from taking too long
          if (iterations >= iterationLimit) {
            t = avg_t;
            break;
          }

          // Calculate new module forces at lerp value avg_t
          var newStates = new SwerveModuleState[4];
          for (int i = 0; i < 4; i++) {
            newStates[i] =
                new SwerveModuleState(
                    MathUtil.interpolate(
                        lastStates[i].speedMetersPerSecond,
                        desiredStates[i].speedMetersPerSecond,
                        avg_t),
                    lastStates[i].angle.interpolate(desiredStates[i].angle, avg_t));
          }
          var newModuleForces =
              getModuleForces(lastChassisSpeeds, kinematics.toChassisSpeeds(newStates));

          // Loop over the new forces to see if there's a constraint violation
          int tooSmallCount = 0;
          int usedModulesCount = 0;
          boolean tooBig = false;
          for (int i = 0; i < 4; i++) {
            // Only count major constraints
            if (Math.abs(maxModuleForceMagnitudesNewtons[i]) / largestMagnitude > .15) {
              usedModulesCount++;
              var desiredForce = newModuleForces.get(i, 0);
              var maxForce = maxModuleForceMagnitudesNewtons[i];
              if (desiredForce / maxForce > 1.01) {
                // Constraint is violated, break the loop
                tooBig = true;
                break;
              } else if (desiredForce / maxForce < 0.99) {
                // If the desired force is smaller than the constraint, it's too small
                tooSmallCount++;
              }
            }
          }
          if (tooBig) {
            // If any of the constraints are violated, avg_t is too big, so we need to search the
            // range between low_t and avg_t
            high_t = avg_t;
          } else if (tooSmallCount == usedModulesCount) {
            // If all the desired module forces are smaller than the constraint, avg_t is too small,
            // we need to search the range between avg_t and high_t
            low_t = avg_t;
          } else {
            // If it's within acceptable limits, we found the largest t value that satisfies the
            // constraint, and break the loop
            t = avg_t;
            break;
          }
          iterations++;
        }
        if (enableLogging) {
          Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Iterations", iterations);
          Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/t", t);
          Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Final high t", high_t);
          Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Final low t", low_t);
        }
        updateDesiredStates(desiredStates, t);
      } else if (enableLogging) {
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Iterations", 0);
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/t", 1.0);
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Final high t", 1.0);
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Final low t", 0.0);
      }
    }

    var finalModuleForces =
        getModuleForces(lastChassisSpeeds, kinematics.toChassisSpeeds(desiredStates));
    var driveFeedforwardAmps = new double[4];
    for (int i = 0; i < 4; i++) {
      driveFeedforwardAmps[i] =
          driveMotor.getCurrent(finalModuleForces.get(i, 0) * wheelRadiusMeters);
    }

    if (enableLogging) {
      var finalModuleForceVisualizations = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        finalModuleForceVisualizations[i] =
            new SwerveModuleState(finalModuleForces.get(i, 0), lastStates[i].angle);
      }
      Logger.recordOutput(
          "Swerve/Drive Calculations/Final Module Forces", finalModuleForceVisualizations);
    }

    // Calculate
    var steerVelocitiesRadPerSec = new double[4];
    for (int i = 0; i < 4; i++) {
      steerVelocitiesRadPerSec[i] =
          desiredStates[i].angle.minus(lastStates[i].angle).getRadians() / Robot.defaultPeriodSecs;
    }

    lastStates = desiredStates;

    if (enableLogging) {
      Logger.recordOutput("Swerve/Drive Calculations/Output/Module States", desiredStates);
      Logger.recordOutput(
          "Swerve/Drive Calculations/Output/Drive Feedforwards Amps", driveFeedforwardAmps);
      Logger.recordOutput(
          "Swerve/Drive Calculations/Output/Steer Feedforwards Rad Per Sec",
          steerVelocitiesRadPerSec);
    }

    return new Setpoint(desiredStates, driveFeedforwardAmps, steerVelocitiesRadPerSec);
  }

  public record Setpoint(
      SwerveModuleState[] states,
      double[] driveFeedforwardsAmps,
      double[] steerVelocitiesRadPerSec) {}

  /**
   * Calculate the module forces needed to accelerate from the last chassis speeds to the new
   * chassis speeds.
   *
   * @return A 4x1 matrix containing the module forces in Newtons.
   */
  private SimpleMatrix getModuleForces(
      ChassisSpeeds lastChassisSpeeds, ChassisSpeeds newChassisSpeeds) {
    return moduleForceIK.mult(
        new SimpleMatrix(
            3,
            1,
            false,
            (newChassisSpeeds.vxMetersPerSecond - lastChassisSpeeds.vxMetersPerSecond)
                / Robot.defaultPeriodSecs
                * robotMassKg
                / 4,
            (newChassisSpeeds.vyMetersPerSecond - lastChassisSpeeds.vyMetersPerSecond)
                / Robot.defaultPeriodSecs
                * robotMassKg
                / 4,
            (newChassisSpeeds.omegaRadiansPerSecond - lastChassisSpeeds.omegaRadiansPerSecond)
                / Robot.defaultPeriodSecs
                * robotMOIKgMetersSquared
                / 4));
  }

  /** Mutate desiredStates based on a linear interpolation value t. */
  private void updateDesiredStates(SwerveModuleState[] desiredStates, double t) {
    for (int i = 0; i < 4; i++) {
      if (t == 0) {
        // We don't need to do the full math, just set desiredStates to lastStates
        desiredStates[i] = lastStates[i];
      } else if (t != 1) {
        // If t = 1, then desiredStates can be untouched, but if t != 1, then we do need to
        // update it
        desiredStates[i] =
            new SwerveModuleState(
                MathUtil.interpolate(
                    lastStates[i].speedMetersPerSecond, desiredStates[i].speedMetersPerSecond, t),
                lastStates[i].angle.interpolate(desiredStates[i].angle, t));
      }
    }
  }

  /**
   * Calculate the max useful stator current draw given the current drive motor velocity.
   *
   * <p>Useful stator current draw does not include free current as that is not contributing towards
   * output torque.
   *
   * @param velRadPerSec The current angular velocity of the wheel.
   * @return The max useful stator current draw.
   */
  private double getMaxTorqueCurrent(double velRadPerSec) {
    // Stator current limit, supply current limit, and motor back-EMF all limit the max torque,
    // so take the min of those
    return Math.min(
        Math.min(
            statorCurrentLimitAmps,
            // Calculate max stator current given the supply limit and the current velocity
            // Original derivation by rafi on ChiefDelphi:
            // https://www.chiefdelphi.com/t/psa-your-motor-curves-are-still-wrong-a-correction-to-a-whitepaper-about-current-limits/504706
            // Slightly modified to ignore free current
            (-driveMotor.stallCurrentAmps * (velRadPerSec / driveMotor.freeSpeedRadPerSec)
                    + Math.sqrt(
                        Math.pow(
                                driveMotor.stallCurrentAmps
                                    * (velRadPerSec / driveMotor.freeSpeedRadPerSec),
                                2)
                            + 4
                                * (RobotController.getBatteryVoltage() / 12)
                                * driveMotor.stallCurrentAmps
                                * supplyCurrentLimitAmps))
                / 2),
        // Don't use DCMotor's calculations since it includes free current which isn't desired
        Math.max(
            driveMotor.stallCurrentAmps * (RobotController.getBatteryVoltage() / 12)
                - driveMotor.stallCurrentAmps * (velRadPerSec / driveMotor.freeSpeedRadPerSec),
            0));
  }
}
