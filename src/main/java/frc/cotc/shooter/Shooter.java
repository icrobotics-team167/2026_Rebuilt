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
  // Time in seconds for the shot's velocity to decay by 1/e times (decay to ~36.8%)
  // TODO: Tune
  private final double DRAG_CONSTANT_INVERSE_SECONDS = 0;

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
        && MathUtil.isNear(lastYawRad, turretInputs.thetaRad, Units.degreesToRadians(1));
  }

  private double lastPitchRad;
  private double lastYawRad;
  private double lastFlywheelVelRotPerSec;

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
    var shooterToTarget = targetLocation.minus(shooterTranslation);

    // Rotation affects the velocity of the shooter, so account for that
    var shooterVx =
        fieldChassisSpeeds.vxMetersPerSecond
            - fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getY();
    var shooterVy =
        fieldChassisSpeeds.vyMetersPerSecond
            + fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getX();

    var projectileSpeedMetersPerSec =
        flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec);

    final int iterations = 10;
    Pose2d[] iterationsPoses = new Pose2d[iterations + 1];
    iterationsPoses[0] = new Pose2d(shooterTranslation, Rotation2d.kZero);
    // TODO: Replace this with Newton's method
    // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/newton-shooting.html
    // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
    var result =
        getResult(
            shooterToTarget.getNorm(),
            clampShotSpeedToBounds(
                shooterToTarget.getNorm(), projectileSpeedMetersPerSec, shotTarget),
            shotTarget);
    var virtualShooterToTarget =
        targetLocation.minus(
            shooterTranslation.plus(
                new Translation2d(
                    shooterVx * result.timeOfFlightSeconds(),
                    shooterVy * result.timeOfFlightSeconds())));
    for (int i = 0; i < iterations; i++) {
      var clampedShotSpeed =
          clampShotSpeedToBounds(
              virtualShooterToTarget.getNorm(), projectileSpeedMetersPerSec, shotTarget);
      var virtualShooterTranslation =
          shooterTranslation.plus(
              new Translation2d(
                  shooterVx * result.timeOfFlightSeconds(),
                  shooterVy * result.timeOfFlightSeconds()));
      virtualShooterToTarget = targetLocation.minus(virtualShooterTranslation);
      result = getResult(virtualShooterToTarget.getNorm(), clampedShotSpeed, shotTarget);
      iterationsPoses[i + 1] = new Pose2d(virtualShooterTranslation, Rotation2d.kZero);
    }
    Logger.recordOutput("Shooter/Shot result/Iterations", iterationsPoses);
    Logger.recordOutput("Shooter/Shot result/Result", result);

    // if (projectileSpeedMetersPerSec < minShotSpeedMetersPerSec) {
    //   flywheelIO.runVel(projectileVelToFlywheelVelMap.get(minShotSpeedMetersPerSec));
    //   projectileSpeedMetersPerSec = minShotSpeedMetersPerSec;
    // } else if (projectileSpeedMetersPerSec > maxShotSpeedMetersPerSec) {
    //   flywheelIO.runVel(projectileVelToFlywheelVelMap.get(maxShotSpeedMetersPerSec));
    //   projectileSpeedMetersPerSec = maxShotSpeedMetersPerSec;
    // } else {
    //   var flywheelAccelerationMetersPerSecSquared =
    //       (flywheelInputs.velRotPerSec - lastFlywheelVelRotPerSec) / Robot.defaultPeriodSecs;
    //   if (flywheelAccelerationMetersPerSecSquared > -1) {
    //     flywheelIO.runVel(lastFlywheelVelRotPerSec);
    //   }
    // }
    // lastFlywheelVelRotPerSec = flywheelInputs.velRotPerSec;

    var turretYawAbsolute = virtualShooterToTarget.getAngle();

    Logger.recordOutput(
        "Shooter/Shot result/Trajectory",
        TrajectoryCalc.simulateShot(
            new Translation3d(
                shooterTranslation.getX(), shooterTranslation.getY(), Units.inchesToMeters(18)),
            new Translation3d(
                    clampShotSpeedToBounds(
                        virtualShooterToTarget.getNorm(), projectileSpeedMetersPerSec, shotTarget),
                    new Rotation3d(0, -result.pitchRad(), turretYawAbsolute.getRadians()))
                .plus(new Translation3d(shooterVx, shooterVy, 0))));

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
        distanceMeters,
        MathUtil.clamp(
            shotSpeedMetersPerSec,
            shotMap.getMinSpeed(distanceMeters),
            shotMap.getMaxSpeed(distanceMeters)));
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
