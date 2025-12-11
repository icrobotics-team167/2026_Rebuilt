// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Robot;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIO.SwerveIOInputs inputs = new SwerveIO.SwerveIOInputs();

  private SwerveDrivePoseEstimator replayPoseEstimator;

  public Swerve(SwerveIO io) {
    this.io = io;

    io.updateInputs(inputs);
    // TODO: Log

    if (Robot.mode == Robot.Mode.REPLAY) {
      replayPoseEstimator =
          new SwerveDrivePoseEstimator(
              new SwerveDriveKinematics(
                  new Translation2d(
                      TunerConstants.FrontLeft.LocationX, TunerConstants.FrontRight.LocationY),
                  new Translation2d(
                      TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                  new Translation2d(
                      TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                  new Translation2d(
                      TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)),
              inputs.currentState.RawHeading,
              inputs.currentState.ModulePositions,
              Pose2d.kZero);
    }
  }

  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault()
          .getTable("Swerve")
          .getStructTopic("Estimated Pose", Pose2d.struct)
          .publish();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // TODO: Log
    if (Robot.mode == Robot.Mode.REPLAY) {
      for (var state : inputs.odoStates) {
        replayPoseEstimator.updateWithTime(
            state.Timestamp, state.RawHeading, state.ModulePositions);
      }
    }
    posePublisher.set(getEstimatedPose());
  }

  private void addVisionMeasurement(
      Pose2d visionPose, double visionTimestamp, Matrix<N3, N1> stdDevs) {
    if (Robot.mode == Robot.Mode.REPLAY) {
      replayPoseEstimator.addVisionMeasurement(
          visionPose, visionTimestamp + inputs.timeOffset, stdDevs);
    } else {
      io.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(visionTimestamp), stdDevs);
    }
  }

  private Pose2d getEstimatedPose() {
    if (Robot.mode == Robot.Mode.REPLAY) {
      return replayPoseEstimator.getEstimatedPosition();
    } else {
      return inputs.currentState.Pose;
    }
  }

  private final double maxLinearSpeedMetersPerSecond =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularSpeedRadiansPerSecond =
      maxLinearSpeedMetersPerSecond
          / Math.min(
              Math.min(
                  Math.hypot(
                      TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                  Math.hypot(
                      TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
              Math.min(
                  Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                  Math.hypot(
                      TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  public Command teleopDrive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return run(
        () ->
            io.drive(
                new ChassisSpeeds(
                    vx.getAsDouble() * maxLinearSpeedMetersPerSecond,
                    vy.getAsDouble() * maxLinearSpeedMetersPerSecond,
                    omega.getAsDouble() * maxAngularSpeedRadiansPerSecond)));
  }
}
