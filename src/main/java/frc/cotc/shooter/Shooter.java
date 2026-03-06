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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants;
import frc.cotc.FieldConstants;
import frc.cotc.Robot;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // private final HoodIO hoodIO;
  private final FlywheelIO flywheelIO;
  // private final TurretIO turretIO;

  // private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldChassisSpeedsSupplier;

  private final InterpolatingDoubleTreeMap distanceToFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

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
      // HoodIO hoodIO,
      FlywheelIO flywheelIO,
      // TurretIO turretIO,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldChassisSpeedsSupplier) {
    // this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;
    // this.turretIO = turretIO;

    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldChassisSpeedsSupplier = fieldChassisSpeedsSupplier;

    // addMapping(0, 0);

  }

  private void addMapping(double flywheelVelRotPerSec, double projectileVelMetersPerSec) {
    flywheelVelToProjectileVelMap.put(flywheelVelRotPerSec, projectileVelMetersPerSec);
    projectileVelToFlywheelVelMap.put(projectileVelMetersPerSec, flywheelVelRotPerSec);
  }

  @Override
  public void periodic() {
    // hoodIO.updateInputs(hoodInputs);
    // Logger.processInputs("Shooter/Hood", hoodInputs);
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    // turretIO.updateInputs(turretInputs);
    // Logger.processInputs("Shooter/Turret", turretInputs);
  }

  // private static final ShotMap hubShotMap = ShotMap.loadFromDeploy("HubShotMap.json");
  // private static final ShotMap groundShotMap = ShotMap.loadFromDeploy("GroundShotMap.json");

  private double idleVel = 50;

  public Command incrementIdleVel() {
    return Commands.runOnce(() -> idleVel += 5);
  }

  public Command decrementIdleVel() {
    return Commands.runOnce(() -> idleVel -= 5);
  }

  public Command idleRun() {
    return run(() -> {
      flywheelIO.runVel(idleVel);
      Logger.recordOutput("Shooter/Flywheel vel", idleVel);
    });
  }

  public enum ShotTarget {
    BLUE_HUB(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
    // BLUE_BOTTOM_GROUND(groundShotMap, BLUE_BOTTOM_GROUND_TARGET),
    // BLUE_TOP_GROUND(groundShotMap, BLUE_TOP_GROUND_TARGET),
    RED_HUB(FieldConstants.Hub.oppTopCenterPoint.toTranslation2d());
    // RED_BOTTOM_GROUND(groundShotMap, RED_BOTTOM_GROUND_TARGET),
    // RED_TOP_GROUND(groundShotMap, RED_TOP_GROUND_TARGET);

    // public final ShotMap map;
    private final Translation2d targetLocation;

    private final double MAX_OFFSET = 0.2;

    public Translation2d getTargetLocation() {
      return targetLocation.plus(
          new Translation2d(
              MAX_OFFSET * Math.sin(5 * Timer.getTimestamp()),
              MAX_OFFSET * Math.sin(8 * Timer.getTimestamp())));
    }

    ShotTarget(Translation2d targetLocation) {
      // this.map = map;
      this.targetLocation = targetLocation;
    }
  }

  public Command shootAt(ShotTarget shotTarget) {
    return run(() -> {
          runShooter(robotPoseSupplier.get(), fieldChassisSpeedsSupplier.get(), shotTarget);
        })
        .withName("Shoot at " + shotTarget.name());
  }

  // public Command pass() {
  //   return run(
  //       () -> {
  //         var blueTargetX = 1.0;
  //         var targetX = Robot.isOnRed() ? FieldConstants.fieldLength - blueTargetX : blueTargetX;
  //         var distance = Math.abs(targetX - robotPoseSupplier.get().getX());
  //         var result =
  //             groundShotMap.get(
  //                 distance, flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec));
  //         // runHood(result.pitchRad());
  //         runFlywheel(groundShotMap, distance);
  //       });
  // }

  // public boolean canShoot() {
  //   return MathUtil.isNear(lastPitchRad, hoodInputs.thetaRad, Units.degreesToRadians(1))
  //       && MathUtil.isNear(lastYawRad, turretInputs.thetaRad, Units.degreesToRadians(1))
  //       && isFlywheelSpeedOk;
  // }

  private double lastPitchRad;
  private double lastYawRad;
  private double lastFlywheelVelRotPerSec;
  private boolean isFlywheelSpeedOk = false;

  private void runShooter(
      Pose2d robotPose, ChassisSpeeds fieldChassisSpeeds, ShotTarget shotTarget) {
    // robotPose =
    //     robotPose.plus(
    //         new Transform2d(
    //             fieldChassisSpeeds.vxMetersPerSecond * LOOK_AHEAD_SECONDS,
    //             fieldChassisSpeeds.vyMetersPerSecond * LOOK_AHEAD_SECONDS,
    //             new Rotation2d(fieldChassisSpeeds.omegaRadiansPerSecond * LOOK_AHEAD_SECONDS)));
    // // Calculate where the shooter is on the field
    // var shooterTranslation = robotPose.plus(robotToShooterTransform).getTranslation();
    // // Rotate the robotToShooterTransform by the robot yaw
    // var robotCenterToShooter = shooterTranslation.minus(robotPose.getTranslation());
    //
    // // Get target location and delta pos from shooter to target
    // var targetLocation = shotTarget.targetLocation;
    //
    // // Rotation affects the velocity of the shooter, so account for that
    // var shooterVx =
    //     fieldChassisSpeeds.vxMetersPerSecond
    //         - fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getY();
    // var shooterVy =
    //     fieldChassisSpeeds.vyMetersPerSecond
    //         + fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getX();
    //
    // var projectileSpeedMetersPerSec =
    //     flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec);
    // var map = shotTarget.map;
    //
    // final int iterations = 10;
    // var iterationsPoses = new Pose2d[iterations + 2];
    // iterationsPoses[0] =
    //     new Pose2d(shooterTranslation, targetLocation.minus(shooterTranslation).getAngle());
    // // Initial guess using the stationary shot's time of flight
    // var baseDistance = targetLocation.getDistance(shooterTranslation);
    // var timeOfFlight = map.get(baseDistance, projectileSpeedMetersPerSec).timeOfFlightSeconds();
    // for (int i = 1; i <= iterations; i++) {
    //   var a =
    //       (1 - Math.exp(-DRAG_CONSTANT_INVERSE_SECONDS * timeOfFlight))
    //           / DRAG_CONSTANT_INVERSE_SECONDS;
    //   var virtualShooterPos =
    //       shooterTranslation.plus(
    //           new Translation2d(shooterVx * timeOfFlight * a, shooterVy * timeOfFlight * a));
    //   var virtualShooterToTarget = targetLocation.minus(virtualShooterPos); // d(τ)
    //   iterationsPoses[i] = new Pose2d(virtualShooterPos, virtualShooterToTarget.getAngle());
    //
    //   var virtualShooterToTargetNorm = virtualShooterToTarget.getNorm(); // D(τ)
    //
    //   var timeOfFlightDerivative =
    //       map.getTimeOfFlightDerivative(
    //           virtualShooterToTargetNorm, projectileSpeedMetersPerSec); // τ'(D(τ))
    //   var a_prime = Math.exp(-DRAG_CONSTANT_INVERSE_SECONDS * timeOfFlight); // a'(τ)
    //
    //   var error =
    //       timeOfFlight
    //           - map.get(virtualShooterToTargetNorm, projectileSpeedMetersPerSec)
    //               .timeOfFlightSeconds(); // E(τ)
    //   var error_prime =
    //       1
    //           + a_prime
    //               * timeOfFlightDerivative
    //               * (virtualShooterToTarget.getX() * shooterVx
    //                   + virtualShooterToTarget.getY() * shooterVy)
    //               / virtualShooterToTargetNorm; // E'(τ)
    //   timeOfFlight -= error / error_prime;
    // }
    // var finalVirtualShooterPos =
    //     shooterTranslation.plus(
    //         new Translation2d(shooterVx * timeOfFlight, shooterVy * timeOfFlight));
    // var finalVirtualShooterToTarget = targetLocation.minus(finalVirtualShooterPos);
    // iterationsPoses[iterations + 1] =
    //     new Pose2d(finalVirtualShooterPos, finalVirtualShooterToTarget.getAngle());
    // var finalVirtualShooterToTargetDistance = finalVirtualShooterToTarget.getNorm();
    // var result = map.get(finalVirtualShooterToTargetDistance, projectileSpeedMetersPerSec);
    // Logger.recordOutput("Shooter/Shot result/Iterations", iterationsPoses);
    // Logger.recordOutput("Shooter/Shot result/Result", result);
    // Logger.recordOutput("Shooter/Shot result/Distance", finalVirtualShooterToTargetDistance);
    // var clampedProjectileSpeed =
    //     map.clampSpeed(finalVirtualShooterToTarget.getNorm(), projectileSpeedMetersPerSec);
    // Logger.recordOutput("Shooter/Shot result/Clamped projectile speed", clampedProjectileSpeed);
    //
    // var turretYawAbsolute = finalVirtualShooterToTarget.getAngle();
    //
    // var shooterPose =
    //     new Pose3d(
    //         shooterTranslation.getX(),
    //         shooterTranslation.getY(),
    //         Units.inchesToMeters(18),
    //         new Rotation3d(0, -result.pitchRad(), turretYawAbsolute.getRadians()));
    // Logger.recordOutput("Shooter/Shot result/Pose", shooterPose);
    // if (Robot.isSimulation()) {
    //   Logger.recordOutput(
    //       "Shooter/Shot result/Trajectory",
    //       TrajectoryCalc.simulateShot(
    //           shooterPose.getTranslation(),
    //           new Translation3d(clampedProjectileSpeed, shooterPose.getRotation())
    //               .plus(new Translation3d(shooterVx, shooterVy, 0))));
    // }

    // runTurret(
    //     turretYawAbsolute,
    //     robotPose.getRotation(),
    //     fieldChassisSpeeds.omegaRadiansPerSecond);
    // var map = shotTarget.map;
    var distanceMeters =
        shotTarget
            .getTargetLocation()
            .getDistance(robotPose.plus(Constants.robotToShooterTransform).getTranslation());
    // var result =
    //     map.get(distanceMeters, flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec));
    //
    // runHood(result.pitchRad());
    // runFlywheel(map, distanceMeters);
    var flywheelSpeed = distanceToFlywheelSpeedMap.get(distanceMeters);
    flywheelIO.runVel(flywheelSpeed);
    isFlywheelSpeedOk =
        flywheelSpeedOkDebouncer.calculate(
            MathUtil.isNear(flywheelSpeed, flywheelInputs.velRotPerSec, 50));
  }

  private final Debouncer flywheelSpeedOkDebouncer =
      new Debouncer(FLYWHEEL_SPEED_OK_DEBOUNCE_SECONDS);

  private void runFlywheel(ShotMap map, double distanceMeters) {
    var projectileSpeedMetersPerSec =
        flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec);
    double minShotSpeedMetersPerSec = map.getMinSpeedMetersPerSec(distanceMeters);
    double maxShotSpeedMetersPerSec = map.getMaxSpeedMetersPerSec(distanceMeters);
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

  // private void runTurret(Rotation2d absoluteYaw, Rotation2d robotYaw, double robotOmegaRadPerSec)
  // {
  //   turretIO.runYaw(
  //       absoluteYaw.minus(robotYaw).getRadians(),
  //       // Feedforward component also includes the robot's motion
  //       (absoluteYaw.getRadians() - lastYawRad) / Robot.defaultPeriodSecs - robotOmegaRadPerSec);
  //   lastYawRad = absoluteYaw.getRadians();
  // }

  // private void runHood(double pitchRad) {
  //   hoodIO.runPitch(pitchRad, (pitchRad - lastPitchRad) / Robot.defaultPeriodSecs);
  //   lastPitchRad = pitchRad;
  // }
}
