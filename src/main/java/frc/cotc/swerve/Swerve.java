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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Robot;
import java.util.function.DoubleSupplier;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;

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

  public Swerve(SwerveIO io) {
    this.io = io;

    var constants = io.getConstants();
    Logger.processInputs("Swerve/Constants", constants);

    var moduleLocations =
        new Translation2d[] {
          new Translation2d(constants.trackLengthMeters / 2, constants.trackWidthMeters / 2),
          new Translation2d(constants.trackLengthMeters / 2, -constants.trackWidthMeters / 2),
          new Translation2d(-constants.trackLengthMeters / 2, constants.trackWidthMeters / 2),
          new Translation2d(-constants.trackLengthMeters / 2, -constants.trackWidthMeters / 2)
        };
    kinematics = new SwerveDriveKinematics(moduleLocations);
    for (int i = 0; i < 4; i++) {
      modPosReciprocals[i] =
          new Translation2d(1.0 / moduleLocations[i].getNorm(), moduleLocations[i].getAngle());
    }

    wheelRadiusMeters = constants.wheelRadiusMeters;
    driveMotor = constants.driveMotor;
    statorCurrentLimitAmps = constants.slipCurrentAmps;
    supplyCurrentLimitAmps = constants.supplyCurrentLimitAmps;
    maxLinearSpeedMetersPerSec = driveMotor.freeSpeedRadPerSec * wheelRadiusMeters;
    maxAngularSpeedRadPerSec =
        maxLinearSpeedMetersPerSec
            / Math.hypot(constants.trackLengthMeters / 2, constants.trackWidthMeters / 2);
    maxDeltaThetaRad = constants.maxSteerSpeedRadPerSec * Robot.defaultPeriodSecs;
    Logger.recordOutput(
        "Swerve/Drive Calculations/Steer Speed/Max delta theta rad", maxDeltaThetaRad);
    robotMassKg = constants.robotMassKg;
    robotMOIKgMetersSquared = constants.robotMOIKgMetersSquared;
    maxDeltaVelMps =
        (4 * constants.driveMotor.KtNMPerAmp * constants.slipCurrentAmps / wheelRadiusMeters)
            / constants.robotMassKg
            * Robot.defaultPeriodSecs;
    Logger.recordOutput(
        "Swerve/Drive Calculations/Traction/Max acceleration",
        maxDeltaVelMps / Robot.defaultPeriodSecs);
  }

  private SwerveModuleState[] lastStates =
      new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.kZero),
        new SwerveModuleState(0, Rotation2d.kZero),
        new SwerveModuleState(0, Rotation2d.kZero),
        new SwerveModuleState(0, Rotation2d.kZero)
      };

  private final double maxDeltaThetaRad;
  private final double maxDeltaVelMps;

  private void calculateNextState(ChassisSpeeds commandedChassisSpeeds) {
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

        Logger.recordOutput(
            "Swerve/Drive Calculations/Optimizations/" + i + "/Delta Vels To Stop", deltaVelToStop);
        Logger.recordOutput(
            "Swerve/Drive Calculations/Optimizations/" + i + "/Delta Vels To Smooth",
            deltaVelToSmooth);
        Logger.recordOutput(
            "Swerve/Drive Calculations/Optimizations/" + i + "/Delta Vels To Smooth Flipped",
            deltaVelToSmoothFlipped);

        // If it's faster to move smoothly, we shouldn't stop first
        if (deltaVelToSmooth < deltaVelToStop + .01
            || deltaVelToSmoothFlipped < deltaVelToStop + .01) {
          allModulesShouldStop = false;
          // If it's faster to flip the direction when moving smoothly, do so
          if (deltaVelToSmoothFlipped < deltaVelToSmooth) {
            desiredStates[i].speedMetersPerSecond *= -1;
            desiredStates[i].angle = desiredStates[i].angle.plus(Rotation2d.kPi);
            Logger.recordOutput(
                "Swerve/Drive Calculations/Optimizations/" + i + "/Faster To:", "Smooth Flipped");
          } else {
            Logger.recordOutput(
                "Swerve/Drive Calculations/Optimizations/" + i + "/Faster To:", "Smooth");
          }
        } else {
          Logger.recordOutput(
              "Swerve/Drive Calculations/Optimizations/" + i + "/Faster To:", "Stop");
        }
      }
      // If it's faster to stop for all modules, then do so
      if (allModulesShouldStop) {
        for (int i = 0; i < 4; i++) {
          desiredStates[i].speedMetersPerSecond = 0;
        }
      }
      Logger.recordOutput(
          "Swerve/Drive Calculations/Optimizations/All Modules Should Stop", allModulesShouldStop);
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
      Logger.recordOutput(
          "Swerve/Drive Calculations/Steer Speed/Desired Delta Thetas Rad", desiredDeltaThetasRad);
      Logger.recordOutput("Swerve/Drive Calculations/Steer Speed/t", t);

      updateDesiredStates(desiredStates, t);
    }

    // Limit acceleration based on max traction
    // This includes sideways acceleration as well in order to minimize wheel wear from slip
    {
      double t = 1;
      double[] desiredDeltaVels = new double[4];
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

      Logger.recordOutput(
          "Swerve/Drive Calculations/Traction/Desired Delta Vels mps", desiredDeltaVels);
      Logger.recordOutput("Swerve/Drive Calculations/Traction/t", t);

      updateDesiredStates(desiredStates, t);

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

    // Limit drive motor dynamics
    // This prevents infeasible velocities/accelerations being commanded
    {
      // Calculate IK and FK for the module forces
      for (int i = 0; i < 4; i++) {
        moduleForceIK.setRow(
            i,
            0,
            lastStates[i].angle.getCos(),
            lastStates[i].angle.getSin(),
            (-modPosReciprocals[i].getY() * lastStates[i].angle.getCos()
                + modPosReciprocals[i].getX() * lastStates[i].angle.getSin()));
      }
      var moduleForceFK = moduleForceIK.pseudoInverse();

      var desiredModuleForces = getModuleForces(lastChassisSpeeds, desiredStates);
      var desiredModuleForceVisualizations = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        desiredModuleForceVisualizations[i] =
            new SwerveModuleState(desiredModuleForces.get(i, 0), lastStates[i].angle);
      }
      Logger.recordOutput(
          "Swerve/Drive Calculations/Motor Dynamics/Desired Module Forces",
          desiredModuleForceVisualizations);

      // Calculate max module forces
      var lastWheelVelsRadPerSec = new double[4];
      var maxStatorCurrents = new double[4];
      var maxModuleForceMagnitudes = new double[4];
      var maxModuleForceVisualizations = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        // Convert m/s to rad/s
        double lastWheelVelRadPerSec = lastStates[i].speedMetersPerSecond / wheelRadiusMeters;
        lastWheelVelsRadPerSec[i] = lastWheelVelRadPerSec;

        // Calculate max stator current given the current velocity and battery voltage
        var maxStatorCurrent =
            desiredModuleForces.get(i, 0) >= 0
                ? getMaxStatorCurrent(lastWheelVelRadPerSec)
                : -getMaxStatorCurrent(-lastWheelVelRadPerSec);
        maxStatorCurrents[i] = maxStatorCurrent;
        // Calculate max module force given the max stator current
        maxModuleForceMagnitudes[i] = maxStatorCurrent * driveMotor.KtNMPerAmp / wheelRadiusMeters;
        maxModuleForceVisualizations[i] =
            new SwerveModuleState(maxModuleForceMagnitudes[i], lastStates[i].angle);
      }
      Logger.recordOutput(
          "Swerve/Drive Calculations/Motor Dynamics/Last Wheel Vel Rad Per Sec",
          lastWheelVelsRadPerSec);
      Logger.recordOutput(
          "Swerve/Drive Calculations/Motor Dynamics/Max Stator Current", maxStatorCurrents);
      Logger.recordOutput(
          "Swerve/Drive Calculations/Motor Dynamics/Max Module Forces",
          maxModuleForceVisualizations);

      double largestMagnitude = 0;
      for (int i = 0; i < 4; i++) {
        largestMagnitude = Math.max(largestMagnitude, Math.abs(maxModuleForceMagnitudes[i]));
      }
      boolean exceededLimit = false;
      for (int i = 0; i < 4; i++) {
        // Small max force constraints (<15% of the largest max force constraint) are ignored
        // as they are relatively negligible and cause issues with slow transitions between
        // states
        // Some amount of kinematic infeasibility is worth it to avoid control weirdness
        if (Math.abs(maxModuleForceMagnitudes[i]) / largestMagnitude > .15) {
          exceededLimit = desiredModuleForces.get(i, 0) / maxModuleForceMagnitudes[i] > 1.01;
          if (exceededLimit) {
            Logger.recordOutput(
                "Swerve/Drive Calculations/Motor Dynamics/Module Exceeding Limits", i);
            break;
          }
        }
      }
      if (!exceededLimit) {
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Module Exceeding Limits", -1);
      }
      Logger.recordOutput(
          "Swerve/Drive Calculations/Motor Dynamics/Largest Magnitude", largestMagnitude);
      if (exceededLimit) {
        double high_t = 1;
        double low_t = 0;
        double t;
        int iterationLimit = 10;
        int iterations = 0;
        while (true) {
          var avg_t = (high_t + low_t) / 2;
          if (iterations >= iterationLimit) {
            t = avg_t;
            break;
          }

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
          var newModuleForces = getModuleForces(lastChassisSpeeds, newStates);
          int tooSmallCount = 0;
          int usedModulesCount = 0;
          boolean tooBig = false;
          for (int i = 0; i < 4; i++) {
            if (Math.abs(maxModuleForceMagnitudes[i]) / largestMagnitude > .15) {
              usedModulesCount++;
              var desiredForce = newModuleForces.get(i, 0);
              var maxForce = maxModuleForceMagnitudes[i];
              if (desiredForce / maxForce > 1.01) {
                tooBig = true;
                break;
              } else if (desiredForce / maxForce < 0.99) {
                tooSmallCount++;
              }
            }
          }
          if (tooBig) {
            high_t = avg_t;
          } else if (tooSmallCount == usedModulesCount) {
            low_t = avg_t;
          } else {
            t = avg_t;
            break;
          }
          iterations++;
        }
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Iterations", iterations);
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/t", t);
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Final high t", high_t);
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Final low t", low_t);
        updateDesiredStates(desiredStates, t);
      } else {
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Iterations", 0);
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/t", 1.0);
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Final high t", 1.0);
        Logger.recordOutput("Swerve/Drive Calculations/Motor Dynamics/Final low t", 0.0);
      }
    }

    var finalModuleForces = getModuleForces(lastChassisSpeeds, desiredStates);
    var finalModuleForceVisualizations = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      finalModuleForceVisualizations[i] =
          new SwerveModuleState(finalModuleForces.get(i, 0), lastStates[i].angle);
    }
    Logger.recordOutput(
        "Swerve/Drive Calculations/Final Module Forces", finalModuleForceVisualizations);

    lastStates = desiredStates;
  }

  private SimpleMatrix getModuleForces(
      ChassisSpeeds lastChassisSpeeds, SwerveModuleState[] newStates) {
    var newChassisSpeeds = kinematics.toChassisSpeeds(newStates);
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

  private void updateDesiredStates(SwerveModuleState[] desiredStates, double t) {
    for (int i = 0; i < 4; i++) {
      if (t == 0) {
        desiredStates[i] = lastStates[i];
      } else if (t != 1) {
        desiredStates[i] =
            new SwerveModuleState(
                MathUtil.interpolate(
                    lastStates[i].speedMetersPerSecond, desiredStates[i].speedMetersPerSecond, t),
                lastStates[i].angle.interpolate(desiredStates[i].angle, t));
      }
    }
  }

  private double getMaxStatorCurrent(double velRadPerSec) {
    return Math.min(
        Math.min(
            statorCurrentLimitAmps,
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
        driveMotor.getCurrent(velRadPerSec, RobotController.getBatteryVoltage()));
  }

  public Command drive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return run(
        () ->
            calculateNextState(
                new ChassisSpeeds(vx.getAsDouble(), vy.getAsDouble(), omega.getAsDouble())));
  }
}
