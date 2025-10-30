// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;

  private final SwerveDriveKinematics kinematics;
  private final CornySetpointGenerator setpointGenerator;

  private final SwerveDrivePoseEstimator poseEstimator;

  public final double maxLinearSpeedMetersPerSec;
  public final double maxAngularSpeedRadPerSec;

  public Swerve(SwerveIO io) {
    this.io = io;

    var constants = io.getConstants();
    Logger.processInputs("Swerve/Constants", constants);

    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);

    var moduleLocations =
        new Translation2d[] {
          new Translation2d(constants.trackLengthMeters / 2, constants.trackWidthMeters / 2),
          new Translation2d(constants.trackLengthMeters / 2, -constants.trackWidthMeters / 2),
          new Translation2d(-constants.trackLengthMeters / 2, constants.trackWidthMeters / 2),
          new Translation2d(-constants.trackLengthMeters / 2, -constants.trackWidthMeters / 2)
        };
    kinematics = new SwerveDriveKinematics(moduleLocations);
    setpointGenerator =
        new CornySetpointGenerator(
            moduleLocations,
            constants.wheelRadiusMeters,
            constants.driveMotor,
            constants.slipCurrentAmps,
            constants.supplyCurrentLimitAmps,
            constants.maxSteerSpeedRadPerSec,
            constants.robotMassKg,
            constants.robotMOIKgMetersSquared,
            inputs.currentStates);
    maxLinearSpeedMetersPerSec = setpointGenerator.maxLinearSpeedMetersPerSec;
    maxAngularSpeedRadPerSec = setpointGenerator.maxAngularSpeedRadPerSec;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            inputs.gyroYaw,
            inputs.odometryFrames[inputs.odometryFrames.length - 1].positions(),
            Pose2d.kZero);
  }

  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);

    for (var frame : inputs.odometryFrames) {
      poseEstimator.updateWithTime(frame.timestampSeconds(), frame.gyroYaw(), frame.positions());
    }
  }

  public Command drive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return run(
        () ->
            io.drive(
                setpointGenerator.calculateNextState(
                    new ChassisSpeeds(vx.getAsDouble(), vy.getAsDouble(), omega.getAsDouble()))));
  }
}
