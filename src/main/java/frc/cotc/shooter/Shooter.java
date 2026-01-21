// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants;
import frc.cotc.Robot;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final HoodIO hoodIO;
  private final FlywheelIO flywheelIO;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final Transform2d robotToShooterTransform = new Transform2d();

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldChassisSpeedsSupplier;

  // The shooter will lag behind, so try to look a little further into the future to compensate
  private final double LOOK_AHEAD_SECONDS = 0;

  // Location that the robot should shoot at for passing balls
  private final Translation2d BLUE_BOTTOM_GROUND_TARGET = new Translation2d(1, 1);
  private final Translation2d BLUE_TOP_GROUND_TARGET =
      new Translation2d(
          BLUE_BOTTOM_GROUND_TARGET.getX(),
          Constants.FIELD_WIDTH_METERS - BLUE_BOTTOM_GROUND_TARGET.getY());
  private final Translation2d RED_BOTTOM_GROUND_TARGET =
      new Translation2d(
          Constants.FIELD_LENGTH_METERS - BLUE_BOTTOM_GROUND_TARGET.getX(),
          BLUE_BOTTOM_GROUND_TARGET.getY());
  private final Translation2d RED_TOP_GROUND_TARGET =
      new Translation2d(
          RED_BOTTOM_GROUND_TARGET.getX(),
          Constants.FIELD_WIDTH_METERS - RED_BOTTOM_GROUND_TARGET.getY());

  private final InterpolatingDoubleTreeMap projectileVelMap = new InterpolatingDoubleTreeMap();

  public Shooter(
      HoodIO hoodIO,
      FlywheelIO flywheelIO,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldChassisSpeedsSupplier) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;

    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldChassisSpeedsSupplier = fieldChassisSpeedsSupplier;

    // TODO: Measure mapping for flywheel vel to projectile vel

    // TODO: Do we have a turret? Cause how to handle the yaw offset for sotm changes wildly between
    // yes turret and no turret
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
  }

  public enum ShotTarget {
    BLUE_HUB,
    BLUE_BOTTOM_GROUND,
    BLUE_TOP_GROUND,
    RED_HUB,
    RED_BOTTOM_GROUND,
    RED_TOP_GROUND;
  }

  public Command shootAtHub() {
    return run(
        () ->
            runShooter(
                robotPoseSupplier.get(),
                fieldChassisSpeedsSupplier.get(),
                Robot.isOnRed() ? ShotTarget.RED_HUB : ShotTarget.BLUE_HUB,
                projectileVelMap.get(flywheelInputs.velRadPerSec)));
  }

  public Command passToAlliance() {
    return run(
        () -> {
          var robotPose = robotPoseSupplier.get();

          ShotTarget target;
          if (robotPose.getY() > Constants.FIELD_WIDTH_METERS / 2) {
            target = Robot.isOnRed() ? ShotTarget.RED_TOP_GROUND : ShotTarget.BLUE_TOP_GROUND;
          } else {
            target = Robot.isOnRed() ? ShotTarget.RED_BOTTOM_GROUND : ShotTarget.BLUE_BOTTOM_GROUND;
          }

          runShooter(
              robotPose,
              fieldChassisSpeedsSupplier.get(),
              target,
              projectileVelMap.get(flywheelInputs.velRadPerSec));
        });
  }

  private void runShooter(
      Pose2d robotPose,
      ChassisSpeeds fieldChassisSpeeds,
      ShotTarget shotTarget,
      double shooterVelMetersPerSecond) {
    // Calculate where the shooter is on the field
    var shooterPose = robotPose.plus(robotToShooterTransform);
    // Rotate the robotToShooterTransform by the robot yaw
    var robotCenterToShooter =
        robotToShooterTransform.plus(new Transform2d(0, 0, robotPose.getRotation()));

    // Get target location and delta pos from shooter to target
    var targetLocation = getTargetLocation(shotTarget);
    var shooterToTarget = targetLocation.minus(shooterPose.getTranslation());

    // Rotation affects the velocity of the shooter, so account for that
    var shooterVx =
        fieldChassisSpeeds.vxMetersPerSecond
            - fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getY();
    var shooterVy =
        fieldChassisSpeeds.vyMetersPerSecond
            + fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getX();

    // Get initial shot solution for a stationary shot
    var result = getShot(shooterToTarget.getNorm(), shooterVelMetersPerSecond, shotTarget);
    int iterations = 5;
    for (int i = 0; i < iterations; i++) {
      // Offset the shooter's position by how far it will move during the time of flight
      shooterToTarget =
          targetLocation.minus(
              shooterPose
                  .getTranslation()
                  .plus(
                      new Translation2d(
                          shooterVx * (result.timeToFlightSeconds() + LOOK_AHEAD_SECONDS),
                          shooterVy * (result.timeToFlightSeconds() + LOOK_AHEAD_SECONDS))));
      // Recalculate the shot solution
      // This will create a new time of flight, which will change the offset above
      // This converges to a stable lookahead point in 3-5 iterations
      result = getShot(shooterToTarget.getNorm(), shooterVelMetersPerSecond, shotTarget);
    }
    // A shot may not be possible due to the shooter velocity being too low. If so, exit
    if (!isPossible(shooterToTarget.getNorm(), shooterVelMetersPerSecond, shotTarget)) {
      return;
    }

    Logger.recordOutput(
        "Shooter/Shooter pose",
        new Pose3d(
            new Translation3d(shooterPose.getX(), shooterPose.getY(), Units.inchesToMeters(20)),
            new Rotation3d(
                // Rotation3d uses +pitch down, but the shooter pitch is done +pitch up
                // 90 degree offset to make the "Axes" model in AScope look prettier
                0, Math.PI / 2 - result.pitchRad(), shooterToTarget.getAngle().getRadians())));
    Logger.recordOutput("Shooter/Shooter distance meters", shooterToTarget.getNorm());
    Logger.recordOutput("Shooter/Shooter pitch rad", result.pitchRad());
    Logger.recordOutput(
        "Shooter/Lookahead target",
        new Pose2d(shooterPose.getTranslation().plus(shooterToTarget), Rotation2d.kZero));

    hoodIO.runPitch(result.pitchRad());
  }

  private Translation2d getTargetLocation(ShotTarget shotTarget) {
    return switch (shotTarget) {
      case BLUE_HUB -> Constants.BLUE_HUB_LOCATION;
      case RED_HUB -> Constants.RED_HUB_LOCATION;
      case BLUE_BOTTOM_GROUND -> BLUE_BOTTOM_GROUND_TARGET;
      case BLUE_TOP_GROUND -> BLUE_TOP_GROUND_TARGET;
      case RED_BOTTOM_GROUND -> RED_BOTTOM_GROUND_TARGET;
      case RED_TOP_GROUND -> RED_TOP_GROUND_TARGET;
    };
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

  private boolean isPossible(
      double distanceMeters, double velocityMetersPerSecond, ShotTarget shotTarget) {
    return switch (shotTarget) {
      case BLUE_HUB, RED_HUB -> hubShotMap.isPossible(distanceMeters, velocityMetersPerSecond);
      default -> groundShotMap.isPossible(distanceMeters, velocityMetersPerSecond);
    };
  }
}
