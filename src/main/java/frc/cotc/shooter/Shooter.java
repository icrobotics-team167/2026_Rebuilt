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

  // The shooter will lag behind, so try to look a little further into the future to compensate
  // TODO: Tune
  @SuppressWarnings("FieldCanBeLocal")
  private final double LOOK_AHEAD_SECONDS = 0;

  // The ball will slow down due to drag as it flies through the air when the robot was moving when
  // it was launched
  // TODO: Tune
  @SuppressWarnings("FieldCanBeLocal")
  private final double DRAG_COMPENSATION_INVERSE_SECONDS = 0.2;

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
              Robot.isOnRed() ? ShotTarget.RED_HUB : ShotTarget.BLUE_HUB,
              flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec));
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

          runShooter(
              robotPose,
              fieldChassisSpeedsSupplier.get(),
              target,
              flywheelVelToProjectileVelMap.get(flywheelInputs.velRotPerSec));
        })
        .withName("Pass to alliance");
  }

  public boolean canShoot() {
    return shotValid
        && MathUtil.isNear(lastPitchRad, hoodInputs.thetaRad, Units.degreesToRadians(1))
        && MathUtil.isNear(lastYawRad, turretInputs.thetaRad, Units.degreesToRadians(1));
  }

  private double lastPitchRad;
  private double lastYawRad;

  private boolean shotValid = false;

  private void runShooter(
      Pose2d robotPose,
      ChassisSpeeds fieldChassisSpeeds,
      ShotTarget shotTarget,
      double shooterVelMetersPerSecond) {
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

    // Get initial shot solution for a stationary shot
    var result = getShot(shooterToTarget.getNorm(), shooterVelMetersPerSecond, shotTarget);
    // As the ball slows down due to drag, the effective distance shrinks
    // This can be approximated with an exponential function
    var timeCompensationSeconds =
        (1 - Math.exp(-result.timeToTargetSeconds() * DRAG_COMPENSATION_INVERSE_SECONDS))
            / DRAG_COMPENSATION_INVERSE_SECONDS;
    int iterations = 5;
    for (int i = 0; i < iterations; i++) {
      // Offset the shooter's position by how far it will move during the time of flight
      shooterToTarget =
          targetLocation.minus(
              shooterTranslation.plus(
                  new Translation2d(
                      shooterVx * (timeCompensationSeconds + LOOK_AHEAD_SECONDS),
                      shooterVy * (timeCompensationSeconds + LOOK_AHEAD_SECONDS))));
      // Recalculate the shot solution
      // This will create a new time of flight, which will change the offset above
      // This converges to a stable lookahead point in 3-5 iterations
      result = getShot(shooterToTarget.getNorm(), shooterVelMetersPerSecond, shotTarget);
      timeCompensationSeconds =
          (1 - Math.exp(-result.timeToTargetSeconds() * DRAG_COMPENSATION_INVERSE_SECONDS))
              / DRAG_COMPENSATION_INVERSE_SECONDS;
    }
    // A shot may not be possible due to the shooter velocity being too low or too high.
    shotValid = isPossible(shooterToTarget.getNorm(), shooterVelMetersPerSecond, shotTarget);

    var turretYawAbsolute = shooterToTarget.getAngle();

    Logger.recordOutput(
        "Shooter/Shooter pose",
        new Pose3d(
            new Translation3d(
                shooterTranslation.getX(), shooterTranslation.getY(), Units.inchesToMeters(20)),
            new Rotation3d(
                // Rotation3d uses +pitch down, but the shooter pitch is done +pitch up
                // 90 degree offset to make the "Axes" model in AScope look prettier
                0, Math.PI / 2 - result.pitchRad(), turretYawAbsolute.getRadians())));
    Logger.recordOutput("Shooter/Shooter distance meters", shooterToTarget.getNorm());
    Logger.recordOutput("Shooter/Shooter pitch rad", result.pitchRad());
    Logger.recordOutput(
        "Shooter/Lookahead target",
        new Pose2d(shooterTranslation.plus(shooterToTarget), Rotation2d.kZero));

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
    flywheelIO.runVel(
        projectileVelToFlywheelVelMap.get(getMaxVelocity(shooterToTarget.getNorm(), shotTarget)));
  }

  private final ShotMap hubShotMap = new HubShotMap();
  private final ShotMap groundShotMap = new GroundShotMap();

  private ShotMap.ShotResult getShot(
      double distanceMeters, double velocityMetersPerSecond, ShotTarget shotTarget) {
    return switch (shotTarget) {
      case BLUE_HUB, RED_HUB -> hubShotMap.get(distanceMeters, velocityMetersPerSecond);
      default -> groundShotMap.get(distanceMeters, velocityMetersPerSecond);
    };
  }

  private final double TOLERANCE_METERS_PER_SEC = 0.01;

  private boolean isPossible(
      double distanceMeters, double velocityMetersPerSecond, ShotTarget shotTarget) {
    return switch (shotTarget) {
      case BLUE_HUB, RED_HUB ->
          hubShotMap.getMinimumVelocity(distanceMeters) - TOLERANCE_METERS_PER_SEC
                  <= velocityMetersPerSecond
              && velocityMetersPerSecond
                  <= hubShotMap.getMaximumVelocity(distanceMeters) + TOLERANCE_METERS_PER_SEC;
      default ->
          groundShotMap.getMinimumVelocity(distanceMeters) - TOLERANCE_METERS_PER_SEC
                  <= velocityMetersPerSecond
              && velocityMetersPerSecond
                  <= groundShotMap.getMaximumVelocity(distanceMeters) + TOLERANCE_METERS_PER_SEC;
    };
  }

  private double getMaxVelocity(double distanceMeters, ShotTarget shotTarget) {
    return switch (shotTarget) {
      case BLUE_HUB, RED_HUB -> hubShotMap.getMaximumVelocity(distanceMeters);
      default -> groundShotMap.getMaximumVelocity(distanceMeters);
    };
  }
}
