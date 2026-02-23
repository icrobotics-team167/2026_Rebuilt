// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.FieldConstants;
import frc.cotc.Robot;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final HoodIO hoodIO;
  private final FlywheelIO flywheelIO;
  private final TurretIO turretIO;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  private final Transform2d robotToShooterTransform = new Transform2d(0.5, 0, Rotation2d.kZero);

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldChassisSpeedsSupplier;

  // The shooter will lag behind the target position, so try to look a little further into the
  // future to compensate
  // TODO: Tune
  @SuppressWarnings("FieldCanBeLocal")
  private final double LOOK_AHEAD_SECONDS = 0;

  // Shooting on the move will induce drag on the projectile, so compensate for that
  // Time in inverse seconds for the shot's velocity to decay by 1/e times (decay to ~36.8%)
  // TODO: Tune
  private final double DRAG_CONSTANT_INVERSE_SECONDS = 0.2;

  private final InterpolatingDoubleTreeMap flywheelVelToProjectileVelMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap projectileVelToFlywheelVelMap =
      new InterpolatingDoubleTreeMap();

  // Location that the robot should shoot at for passing balls
  private final Translation2d BLUE_BOTTOM_GROUND_TARGET = new Translation2d(1, 1);
  private final Translation2d BLUE_TOP_GROUND_TARGET =
      new Translation2d(
          BLUE_BOTTOM_GROUND_TARGET.getX(),
          FieldConstants.fieldWidth - BLUE_BOTTOM_GROUND_TARGET.getY());
  private final Translation2d RED_BOTTOM_GROUND_TARGET =
      new Translation2d(
          FieldConstants.fieldLength - BLUE_BOTTOM_GROUND_TARGET.getX(),
          BLUE_BOTTOM_GROUND_TARGET.getY());
  private final Translation2d RED_TOP_GROUND_TARGET =
      new Translation2d(
          RED_BOTTOM_GROUND_TARGET.getX(),
          FieldConstants.fieldWidth - RED_BOTTOM_GROUND_TARGET.getY());

  private final ShotMap hubShotMap;
  private final ShotMap groundShotMap;

  public Shooter(
      HoodIO hoodIO,
      FlywheelIO flywheelIO,
      TurretIO turretIO,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldChassisSpeedsSupplier) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;
    this.turretIO = turretIO;

    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldChassisSpeedsSupplier = fieldChassisSpeedsSupplier;

    hubShotMap = ShotMap.loadFromDeploy("HubShotMap.json");
    groundShotMap = ShotMap.loadFromDeploy("GroundShotMap.json");

    addMapping(0, 0);
  }

  private void addMapping(double flywheelVelRotPerSec, double projectileVelMetersPerSec) {
    flywheelVelToProjectileVelMap.put(flywheelVelRotPerSec, projectileVelMetersPerSec);
    projectileVelToFlywheelVelMap.put(projectileVelMetersPerSec, flywheelVelRotPerSec);
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Shooter/Turret", turretInputs);
  }

  public enum ShotTarget {
    BLUE_HUB,
    BLUE_BOTTOM_GROUND,
    BLUE_TOP_GROUND,
    RED_HUB,
    RED_BOTTOM_GROUND,
    RED_TOP_GROUND;
  }

  private Translation2d getTargetLocation(ShotTarget shotTarget) {
    return switch (shotTarget) {
      case BLUE_HUB -> FieldConstants.Hub.topCenterPoint.toTranslation2d();
      case RED_HUB -> FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
      case BLUE_BOTTOM_GROUND -> BLUE_BOTTOM_GROUND_TARGET;
      case BLUE_TOP_GROUND -> BLUE_TOP_GROUND_TARGET;
      case RED_BOTTOM_GROUND -> RED_BOTTOM_GROUND_TARGET;
      case RED_TOP_GROUND -> RED_TOP_GROUND_TARGET;
    };
  }

  public Command shootAtHub() {
    return run(() -> {
          runShooter(
              robotPoseSupplier.get(),
              fieldChassisSpeedsSupplier.get(),
              Robot.isOnRed() ? ShotTarget.RED_HUB : ShotTarget.BLUE_HUB);
        })
        .withName("Shoot at hub");
  }

  public Command passToAlliance() {
    return run(() -> {
          var robotPose = robotPoseSupplier.get();

          ShotTarget target;
          if (robotPose.getY() > FieldConstants.fieldWidth / 2) {
            target = Robot.isOnRed() ? ShotTarget.RED_TOP_GROUND : ShotTarget.BLUE_TOP_GROUND;
          } else {
            target = Robot.isOnRed() ? ShotTarget.RED_BOTTOM_GROUND : ShotTarget.BLUE_BOTTOM_GROUND;
          }

          runShooter(robotPose, fieldChassisSpeedsSupplier.get(), target);
        })
        .withName("Pass to alliance");
  }

  public boolean canShoot() {
    return MathUtil.isNear(lastPitchRad, hoodInputs.thetaRad, Units.degreesToRadians(1))
        && MathUtil.isNear(lastYawRad, turretInputs.thetaRad, Units.degreesToRadians(1))
        && isFlywheelSpeedOk;
  }

  private double lastPitchRad;
  private double lastYawRad;
  private double lastFlywheelVelRotPerSec;
  private boolean isFlywheelSpeedOk = false;

  private void runShooter(
      Pose2d robotPose, ChassisSpeeds fieldChassisSpeeds, ShotTarget shotTarget) {
    robotPose =
        robotPose.plus(
            new Transform2d(
                fieldChassisSpeeds.vxMetersPerSecond * LOOK_AHEAD_SECONDS,
                fieldChassisSpeeds.vyMetersPerSecond * LOOK_AHEAD_SECONDS,
                new Rotation2d(fieldChassisSpeeds.omegaRadiansPerSecond * LOOK_AHEAD_SECONDS)));
    // Calculate where the shooter is on the field
    var shooterTranslation = robotPose.plus(robotToShooterTransform).getTranslation();
    // Rotate the robotToShooterTransform by the robot yaw
    var robotCenterToShooter = shooterTranslation.minus(robotPose.getTranslation());

    // Get target location and delta pos from shooter to target
    var targetLocation = getTargetLocation(shotTarget);

    // Rotation affects the velocity of the shooter, so account for that
    var shooterVx =
        fieldChassisSpeeds.vxMetersPerSecond
            - fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getY();
    var shooterVy =
        fieldChassisSpeeds.vyMetersPerSecond
            + fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getX();

    var projectileSpeedMetersPerSec =
        flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec);

    // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/newton-shooting.html
    // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
    // Newton's method iteration for shoot on the move solving
    // Let τ be the time of flight of the projectile
    // Let r be the position of the robot's shooter
    // Let g be the position of the goal
    // Let v be the vector of the shooter's velocity
    // Let d be the vector of the delta between the shooter and the target, compensated by time
    // of flight
    // Let D be the magnitude of d
    // Let k be the linear drag speed decay factor for when the shooter is moving
    //
    // a(τ) = (1 - e^-kτ) / k
    // d(τ) = g - (p + v * a(τ))
    // D(τ) = |d(τ)| = √(d_x(τ)² + d_y(τ)²)
    //
    // Let E(τ) = τ − τ(D(τ)) be the root-finding problem that Newton's method solves for.
    // At E(τ*) = 0, τ* represents the time of flight of the projectile at the solution for the
    // shot parameters, such that using d(τ*) for our shot distance calculations will make it
    // into the goal.
    //
    // Newton's method states:
    // τ_(n+1) = τ_n - E(τ_n) / E'(τ_n)
    //
    // a'(τ) = e^-kt
    // E'(τ) = 1 - τ'(D) * dD/dτ = 1 + a'(τ) * τ'(D) * (d_x * v_x + d_y * v_y) / D

    final int iterations = 10;
    var iterationsPoses = new Pose2d[iterations + 2];
    iterationsPoses[0] =
        new Pose2d(shooterTranslation, targetLocation.minus(shooterTranslation).getAngle());
    // Initial guess using the stationary shot's time of flight
    var baseDistance = targetLocation.getDistance(shooterTranslation);
    var timeOfFlight =
        getResult(baseDistance, projectileSpeedMetersPerSec, shotTarget).timeOfFlightSeconds();
    for (int i = 1; i <= iterations; i++) {
      var a =
          (1 - Math.exp(-DRAG_CONSTANT_INVERSE_SECONDS * timeOfFlight))
              / DRAG_CONSTANT_INVERSE_SECONDS;
      var virtualShooterPos =
          shooterTranslation.plus(
              new Translation2d(shooterVx * timeOfFlight * a, shooterVy * timeOfFlight * a));
      var virtualShooterToTarget = targetLocation.minus(virtualShooterPos); // d(τ)
      iterationsPoses[i] = new Pose2d(virtualShooterPos, virtualShooterToTarget.getAngle());

      var virtualShooterToTargetNorm = virtualShooterToTarget.getNorm(); // D(τ)

      var timeOfFlightDerivative =
          getTimeOfFlightDerivative(
              virtualShooterToTargetNorm, projectileSpeedMetersPerSec, shotTarget); // τ'(D(τ))
      var a_prime = Math.exp(-DRAG_CONSTANT_INVERSE_SECONDS * timeOfFlight); // a'(τ)

      var error =
          timeOfFlight
              - getResult(virtualShooterToTargetNorm, projectileSpeedMetersPerSec, shotTarget)
                  .timeOfFlightSeconds(); // E(τ)
      var error_prime =
          1
              + a_prime
                  * timeOfFlightDerivative
                  * (virtualShooterToTarget.getX() * shooterVx
                      + virtualShooterToTarget.getY() * shooterVy)
                  / virtualShooterToTargetNorm; // E'(τ)
      timeOfFlight -= error / error_prime;
    }
    var finalVirtualShooterPos =
        shooterTranslation.plus(
            new Translation2d(shooterVx * timeOfFlight, shooterVy * timeOfFlight));
    var finalVirtualShooterToTarget = targetLocation.minus(finalVirtualShooterPos);
    iterationsPoses[iterations + 1] =
        new Pose2d(finalVirtualShooterPos, finalVirtualShooterToTarget.getAngle());
    var finalVirtualShooterToTargetDistance = finalVirtualShooterToTarget.getNorm();
    var result =
        getResult(finalVirtualShooterToTargetDistance, projectileSpeedMetersPerSec, shotTarget);
    Logger.recordOutput("Shooter/Shot result/Iterations", iterationsPoses);
    Logger.recordOutput("Shooter/Shot result/Result", result);
    Logger.recordOutput("Shooter/Shot result/Distance", finalVirtualShooterToTargetDistance);
    var clampedProjectileSpeed =
        clampShotSpeedToBounds(
            finalVirtualShooterToTarget.getNorm(), projectileSpeedMetersPerSec, shotTarget);
    Logger.recordOutput("Shooter/Shot result/Clamped projectile speed", clampedProjectileSpeed);

    var minShotSpeedMetersPerSec = groundShotMap.getMinSpeed(finalVirtualShooterToTargetDistance);
    var maxShotSpeedMetersPerSec = groundShotMap.getMaxSpeed(finalVirtualShooterToTargetDistance);
    var tolerance = .5;
    if (projectileSpeedMetersPerSec < minShotSpeedMetersPerSec - tolerance) {
      flywheelIO.runVel(projectileVelToFlywheelVelMap.get(minShotSpeedMetersPerSec));
      isFlywheelSpeedOk = false;
      lastFlywheelVelRotPerSec = flywheelInputs.velRotPerSec;
    } else if (projectileSpeedMetersPerSec > maxShotSpeedMetersPerSec + tolerance) {
      flywheelIO.runVel(projectileVelToFlywheelVelMap.get(maxShotSpeedMetersPerSec));
      isFlywheelSpeedOk = false;
      lastFlywheelVelRotPerSec = flywheelInputs.velRotPerSec;
    } else {
      var flywheelAccelerationMetersPerSecSquared =
          (flywheelInputs.velRotPerSec - lastFlywheelVelRotPerSec) / Robot.defaultPeriodSecs;
      if (flywheelAccelerationMetersPerSecSquared > -tolerance) {
        flywheelIO.runVel(lastFlywheelVelRotPerSec);
      } else {
        lastFlywheelVelRotPerSec = flywheelInputs.velRotPerSec;
      }
      isFlywheelSpeedOk = true;
    }

    var turretYawAbsolute = finalVirtualShooterToTarget.getAngle();

    var shooterPose =
        new Pose3d(
            shooterTranslation.getX(),
            shooterTranslation.getY(),
            Units.inchesToMeters(18),
            new Rotation3d(0, -result.pitchRad(), turretYawAbsolute.getRadians()));
    Logger.recordOutput("Shooter/Shot result/Pose", shooterPose);

    if (Robot.isSimulation()) {
      Logger.recordOutput(
          "Shooter/Shot result/Trajectory",
          TrajectoryCalc.simulateShot(
              shooterPose.getTranslation(),
              new Translation3d(clampedProjectileSpeed, shooterPose.getRotation())
                  .plus(new Translation3d(shooterVx, shooterVy, 0))));
    }

    hoodIO.runPitch(
        result.pitchRad(), (result.pitchRad() - lastPitchRad) / Robot.defaultPeriodSecs);
    lastPitchRad = result.pitchRad();
    turretIO.runYaw(
        // Convert from absolute yaw to robot-relative yaw
        turretYawAbsolute.minus(robotPose.getRotation()).getRadians(),
        // Feedforward component also includes the robot's motion
        (turretYawAbsolute.getRadians() - lastYawRad) / Robot.defaultPeriodSecs
            - fieldChassisSpeeds.omegaRadiansPerSecond);
    lastYawRad = turretYawAbsolute.getRadians();
  }

  private ShotMap.ShotResult getResult(
      double distanceMeters, double shotSpeedMetersPerSec, ShotTarget target) {
    var shotMap =
        switch (target) {
          case BLUE_HUB, RED_HUB -> hubShotMap;
          default -> groundShotMap;
        };
    return shotMap.get(
        distanceMeters, clampShotSpeedToBounds(distanceMeters, shotSpeedMetersPerSec, target));
  }

  private double getTimeOfFlightDerivative(
      double distanceMeters, double shotSpeedMetersPerSec, ShotTarget target) {
    var shotMap =
        switch (target) {
          case BLUE_HUB, RED_HUB -> hubShotMap;
          default -> groundShotMap;
        };
    return shotMap.getTimeOfFlightDerivative(
        distanceMeters, clampShotSpeedToBounds(distanceMeters, shotSpeedMetersPerSec, target));
  }

  private double clampShotSpeedToBounds(
      double distanceMeters, double shotSpeedMetersPerSec, ShotTarget target) {
    var shotMap =
        switch (target) {
          case BLUE_HUB, RED_HUB -> hubShotMap;
          default -> groundShotMap;
        };
    return MathUtil.clamp(
        shotSpeedMetersPerSec,
        shotMap.getMinSpeed(distanceMeters),
        shotMap.getMaxSpeed(distanceMeters));
  }
}
