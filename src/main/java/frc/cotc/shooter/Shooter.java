// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
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

  // Tolerancing and debouncing for the "is flywheel speed ok check"
  private final double FLYWHEEL_ACCELERATION_TOLERANCE_MPSS = 0.5;
  private final double FLYWHEEL_SPEED_TOLERANCE_MPS = 0.5;
  private final double FLYWHEEL_SPEED_OK_DEBOUNCE_SECONDS = 0.2;

  // Location that the robot should shoot at for passing balls
  private static final Translation2d BLUE_BOTTOM_GROUND_TARGET = new Translation2d(1, 1);
  private static final Translation2d BLUE_TOP_GROUND_TARGET =
      new Translation2d(
          BLUE_BOTTOM_GROUND_TARGET.getX(),
          FieldConstants.fieldWidth - BLUE_BOTTOM_GROUND_TARGET.getY());
  private static final Translation2d RED_BOTTOM_GROUND_TARGET =
      new Translation2d(
          FieldConstants.fieldLength - BLUE_BOTTOM_GROUND_TARGET.getX(),
          BLUE_BOTTOM_GROUND_TARGET.getY());
  private static final Translation2d RED_TOP_GROUND_TARGET =
      new Translation2d(
          RED_BOTTOM_GROUND_TARGET.getX(),
          FieldConstants.fieldWidth - RED_BOTTOM_GROUND_TARGET.getY());

  private final InterpolatingDoubleTreeMap flywheelVelToProjectileVelMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap projectileVelToFlywheelVelMap =
      new InterpolatingDoubleTreeMap();

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

  private static final ShotMap hubShotMap = ShotMap.loadFromDeploy("HubShotMap.json");
  private static final ShotMap groundShotMap = ShotMap.loadFromDeploy("GroundShotMap.json");

  public enum ShotTarget {
    BLUE_HUB(hubShotMap, FieldConstants.Hub.topCenterPoint.toTranslation2d()),
    BLUE_BOTTOM_GROUND(groundShotMap, BLUE_BOTTOM_GROUND_TARGET),
    BLUE_TOP_GROUND(groundShotMap, BLUE_TOP_GROUND_TARGET),
    RED_HUB(hubShotMap, FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()),
    RED_BOTTOM_GROUND(groundShotMap, RED_BOTTOM_GROUND_TARGET),
    RED_TOP_GROUND(groundShotMap, RED_TOP_GROUND_TARGET);

    final ShotMap map;
    final Translation2d targetLocation;

    ShotTarget(ShotMap map, Translation2d targetLocation) {
      this.map = map;
      this.targetLocation = targetLocation;
    }
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
    var targetLocation = shotTarget.targetLocation;

    // Rotation affects the velocity of the shooter, so account for that
    var shooterVx =
        fieldChassisSpeeds.vxMetersPerSecond
            - fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getY();
    var shooterVy =
        fieldChassisSpeeds.vyMetersPerSecond
            + fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getX();

    var projectileSpeedMetersPerSec =
        flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec);
    var map = shotTarget.map;

    // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/newton-shooting.html
    // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
    // TODO: Remove this when I get the latex doc done
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
    // a = (1 - e^-kτ) / k
    // d = g - (p + v * a)
    // D = |d(τ)| = √(d_x² + d_y²)
    //
    // Let E = τ − τ(D(τ)) be the root-finding problem that Newton's method solves for.
    // At E = 0, τ represents the time of flight of the projectile at the solution for the
    // shot parameters, such that using d(τ) for our shot distance calculations will make it
    // into the goal.
    //
    // Newton's method states:
    // τ_(n+1) = τ_n - E_n / E'_n
    //
    // a'_n = e^-kt
    // E'_n = 1 - τ'_n * dD/dτ = 1 + a'_n * τ'_n * (d_xn * v_x + d_yn * v_y) / D_n
    final int iterations = 10;
    var iterationsPoses = new Pose2d[iterations + 2];
    iterationsPoses[0] =
        new Pose2d(shooterTranslation, targetLocation.minus(shooterTranslation).getAngle());
    // Initial guess using the stationary shot's time of flight
    var baseDistance = targetLocation.getDistance(shooterTranslation);
    var timeOfFlight = map.get(baseDistance, projectileSpeedMetersPerSec).timeOfFlightSeconds();
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
          map.getTimeOfFlightDerivative(
              virtualShooterToTargetNorm, projectileSpeedMetersPerSec); // τ'(D(τ))
      var a_prime = Math.exp(-DRAG_CONSTANT_INVERSE_SECONDS * timeOfFlight); // a'(τ)

      var error =
          timeOfFlight
              - map.get(virtualShooterToTargetNorm, projectileSpeedMetersPerSec)
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
    var result = map.get(finalVirtualShooterToTargetDistance, projectileSpeedMetersPerSec);
    Logger.recordOutput("Shooter/Shot result/Iterations", iterationsPoses);
    Logger.recordOutput("Shooter/Shot result/Result", result);
    Logger.recordOutput("Shooter/Shot result/Distance", finalVirtualShooterToTargetDistance);
    var clampedProjectileSpeed =
        map.clampSpeed(finalVirtualShooterToTarget.getNorm(), projectileSpeedMetersPerSec);
    Logger.recordOutput("Shooter/Shot result/Clamped projectile speed", clampedProjectileSpeed);

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

    runTurret(
        turretYawAbsolute,
        robotPose.getRotation(),
        fieldChassisSpeeds.omegaRadiansPerSecond,
        result.pitchRad());
    runFlywheel(map);
  }

  private final Debouncer flywheelSpeedOkDebouncer =
      new Debouncer(FLYWHEEL_SPEED_OK_DEBOUNCE_SECONDS);

  private void runFlywheel(ShotMap map) {
    var projectileSpeedMetersPerSec =
        flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec);
    double minShotSpeedMetersPerSec = map.getMinSpeedMetersPerSec(projectileSpeedMetersPerSec);
    double maxShotSpeedMetersPerSec = map.getMaxSpeedMetersPerSec(projectileSpeedMetersPerSec);
    if (flywheelSpeedOkDebouncer.calculate(
        projectileSpeedMetersPerSec < minShotSpeedMetersPerSec - FLYWHEEL_SPEED_TOLERANCE_MPS)) {
      // If the flywheel speed is too slow, raise the speed
      flywheelIO.runVel(projectileVelToFlywheelVelMap.get(minShotSpeedMetersPerSec));
      isFlywheelSpeedOk = false;
      lastFlywheelVelRotPerSec = flywheelInputs.velRotPerSec;
    } else if (flywheelSpeedOkDebouncer.calculate(
        projectileSpeedMetersPerSec > maxShotSpeedMetersPerSec + FLYWHEEL_SPEED_TOLERANCE_MPS)) {
      // If the flywheel speed is too fast, lower the speed
      flywheelIO.runVel(projectileVelToFlywheelVelMap.get(maxShotSpeedMetersPerSec));
      isFlywheelSpeedOk = false;
      lastFlywheelVelRotPerSec = flywheelInputs.velRotPerSec;
    } else {
      // If the flywheel is decelerating, a ball is going through, so keep the target speed the
      // same to let the PID apply a consistent torque to the flywheel
      // If the flywheel isn't decelerating, we're at idle, maintain current target speed.
      var flywheelAccelerationMetersPerSecSquared =
          (flywheelInputs.velRotPerSec - lastFlywheelVelRotPerSec) / Robot.defaultPeriodSecs;
      if (flywheelAccelerationMetersPerSecSquared > -FLYWHEEL_ACCELERATION_TOLERANCE_MPSS) {
        flywheelIO.runVel(lastFlywheelVelRotPerSec);
      } else {
        lastFlywheelVelRotPerSec = flywheelInputs.velRotPerSec;
      }
      isFlywheelSpeedOk = true;
    }
  }

  private void runTurret(
      Rotation2d absoluteYaw, Rotation2d robotYaw, double robotOmegaRadPerSec, double pitchRad) {
    turretIO.runYaw(
        absoluteYaw.minus(robotYaw).getRadians(),
        // Feedforward component also includes the robot's motion
        (absoluteYaw.getRadians() - lastYawRad) / Robot.defaultPeriodSecs - robotOmegaRadPerSec);
    lastYawRad = absoluteYaw.getRadians();
    hoodIO.runPitch(pitchRad, (pitchRad - lastPitchRad) / Robot.defaultPeriodSecs);
    lastPitchRad = pitchRad;
  }
}
