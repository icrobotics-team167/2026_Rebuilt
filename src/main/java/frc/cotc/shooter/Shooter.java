// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants;
import java.util.ArrayList;
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

  private final double LOOK_AHEAD_SECONDS = 0;

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

  public Shooter(
      HoodIO hoodIO,
      FlywheelIO flywheelIO,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldChassisSpeedsSupplier) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;

    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldChassisSpeedsSupplier = fieldChassisSpeedsSupplier;
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    runShooter(
        robotPoseSupplier.get(),
        fieldChassisSpeedsSupplier.get(),
        ShotTarget.BLUE_BOTTOM_GROUND,
        12);
  }

  public enum ShotTarget {
    BLUE_HUB,
    BLUE_BOTTOM_GROUND,
    BLUE_TOP_GROUND,
    RED_HUB,
    RED_BOTTOM_GROUND,
    RED_TOP_GROUND;
  }

  private void runShooter(
      Pose2d robotPose,
      ChassisSpeeds fieldChassisSpeeds,
      ShotTarget shotTarget,
      double shooterVelMetersPerSecond) {
    var shooterPose = robotPose.plus(robotToShooterTransform);
    var robotCenterToShooter =
        robotToShooterTransform.plus(new Transform2d(0, 0, robotPose.getRotation()));

    var targetLocation = getTargetLocation(shotTarget);
    var shooterToTarget = targetLocation.minus(shooterPose.getTranslation());

    var shooterVx =
        fieldChassisSpeeds.vxMetersPerSecond
            - fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getY();
    var shooterVy =
        fieldChassisSpeeds.vyMetersPerSecond
            + fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getX();

    var result = getShot(shooterToTarget.getNorm(), shooterVelMetersPerSecond, shotTarget);
    int iterations = 5;
    var iterationTargets = new ArrayList<Pose2d>();
    for (int i = 0; i < iterations; i++) {
      shooterToTarget =
          targetLocation.minus(
              shooterPose
                  .getTranslation()
                  .plus(
                      new Translation2d(
                          shooterVx * (result.timeToFlightSeconds() + LOOK_AHEAD_SECONDS),
                          shooterVy * (result.timeToFlightSeconds() + LOOK_AHEAD_SECONDS))));
      iterationTargets.add(
          new Pose2d(shooterPose.getTranslation().plus(shooterToTarget), Rotation2d.kZero));
      result = getShot(shooterToTarget.getNorm(), shooterVelMetersPerSecond, shotTarget);
    }
    if (!isPossible(shooterToTarget.getNorm(), shooterVelMetersPerSecond, shotTarget)) {
      return;
    }
    Logger.recordOutput("Shooter/Iteration targets", iterationTargets.toArray(new Pose2d[0]));

    Logger.recordOutput(
        "Shooter/Shooter pose",
        new Pose3d(
            new Translation3d(shooterPose.getX(), shooterPose.getY(), Units.inchesToMeters(20)),
            new Rotation3d(
                0, Math.PI / 2 - result.pitchRad(), shooterToTarget.getAngle().getRadians())));
    Logger.recordOutput("Shooter/Shooter distance", shooterToTarget.getNorm());
    Logger.recordOutput(
        "Shooter/Lookahead target",
        new Pose2d(shooterPose.getTranslation().plus(shooterToTarget), Rotation2d.kZero));
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
