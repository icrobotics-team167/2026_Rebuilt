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
import frc.cotc.Robot;
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
    runShooter(robotPoseSupplier.get(), fieldChassisSpeedsSupplier.get());
  }

  private void runShooter(Pose2d robotPose, ChassisSpeeds fieldChassisSpeeds) {
    var shooterPose = robotPose.plus(robotToShooterTransform);
    var robotCenterToShooter =
        robotToShooterTransform.plus(new Transform2d(0, 0, robotPose.getRotation()));

    var hubLocation = Robot.isOnRed() ? Constants.RED_HUB_LOCATION : Constants.BLUE_HUB_LOCATION;
    var shooterToTarget = hubLocation.minus(shooterPose.getTranslation());

    var shooterVx =
        fieldChassisSpeeds.vxMetersPerSecond
            - fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getY();
    var shooterVy =
        fieldChassisSpeeds.vyMetersPerSecond
            + fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getX();

    var result = HubShotMap.get(shooterToTarget.getNorm(), 15);
    int iterations = 10;
    var iterationTargets = new ArrayList<Pose2d>();
    if (result == null) {
      return;
    }
    for (int i = 0; i < iterations; i++) {
      shooterToTarget =
          hubLocation.minus(
              shooterPose
                  .getTranslation()
                  .plus(
                      new Translation2d(
                          shooterVx * (result.timeToFlightSeconds() + LOOK_AHEAD_SECONDS),
                          shooterVy * (result.timeToFlightSeconds() + LOOK_AHEAD_SECONDS))));
      iterationTargets.add(
          new Pose2d(shooterPose.getTranslation().plus(shooterToTarget), Rotation2d.kZero));
      result = HubShotMap.get(shooterToTarget.getNorm(), 15);
      if (result == null) {
        return;
      }
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
}
