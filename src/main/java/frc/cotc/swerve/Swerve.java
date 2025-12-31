// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants;
import frc.cotc.Robot;
import frc.cotc.vision.AprilTagPoseEstimator;
import frc.cotc.vision.AprilTagPoseEstimatorIOPhoton;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  private final Alert[] deviceDisconnectAlerts = new Alert[12];

  AprilTagPoseEstimator[] cameras;

  @SuppressWarnings("resource")
  public Swerve(SwerveIO io, AprilTagPoseEstimator... cameras) {
    this.io = io;

    this.cameras = cameras;

    // capture initial state and set up odometry
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    io.updateOdometry(inputs);

    final String[] names = new String[] {"Front Left", "Front Right", "Back Left", "Back Right"};
    for (int i = 0; i < 4; i++) {
      deviceDisconnectAlerts[i * 3] =
          new Alert(
              Constants.MOTOR_DISCONNECT_ALERT_GROUP,
              names[i] + " Drive Disconnected",
              Alert.AlertType.kError);
      deviceDisconnectAlerts[i * 3 + 1] =
          new Alert(
              Constants.MOTOR_DISCONNECT_ALERT_GROUP,
              names[i] + " Steer Disconnected",
              Alert.AlertType.kError);
      deviceDisconnectAlerts[i * 3 + 2] =
          new Alert(
              Constants.MOTOR_DISCONNECT_ALERT_GROUP,
              names[i] + " Disconnected",
              Alert.AlertType.kError);
    }
  }

  private final ArrayList<Pose2d> visionPoses = new ArrayList<>();

  interface FakeVisionPoseIO {
    @AutoLog
    class FakeVisionPoseIOInputs {
      public double timestamp = 0;
      public Pose2d pose = Pose2d.kZero;
    }

    default void updateInputs(FakeVisionPoseIOInputs inputs) {}
  }

  private final FakeVisionPoseIO fakeVisionIO =
      Robot.mode == Robot.Mode.REPLAY
          ? new FakeVisionPoseIO() {}
          : new FakeVisionPoseIO() {
            @Override
            public void updateInputs(FakeVisionPoseIOInputs inputs) {
              inputs.timestamp = Timer.getFPGATimestamp();
              inputs.pose = Robot.groundTruthPoseSupplier.get();
            }
          };
  private final FakeVisionPoseIOInputsAutoLogged fakeVisionPoseIOInputsAutoLogged =
      new FakeVisionPoseIOInputsAutoLogged();
  private final LoggedNetworkBoolean fakeVision =
      new LoggedNetworkBoolean("Swerve/Fake Vision Enabled", false);

  @Override
  public void periodic() {
    // Update and process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    // Update odometry using those inputs
    io.updateOdometry(inputs);

    if (Robot.mode == Robot.Mode.SIM) {
      AprilTagPoseEstimatorIOPhoton.updateSim();
    }
    if (fakeVision.get()) {
      fakeVisionIO.updateInputs(fakeVisionPoseIOInputsAutoLogged);
      Logger.processInputs("Swerve/FakeVision", fakeVisionPoseIOInputsAutoLogged);
      io.addVisionMeasurement(
          fakeVisionPoseIOInputsAutoLogged.pose,
          fakeVisionPoseIOInputsAutoLogged.timestamp + inputs.timeOffsetSeconds,
          VecBuilder.fill(0.5, 0.5, 0.5));
    } else {
      for (var camera : cameras) {
        camera.update(
          (Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) -> {
            visionPoses.add(pose);
            io.addVisionMeasurement(pose, timestamp + inputs.timeOffsetSeconds, stdDevs);
          });
      }
      Logger.recordOutput("Swerve/Vision Poses", visionPoses.toArray(new Pose2d[0]));
      visionPoses.clear();
    }

    for (int i = 0; i < 4; i++) {
      deviceDisconnectAlerts[i * 3].set(inputs.driveMotorConnected[i]);
      deviceDisconnectAlerts[i * 3 + 1].set(inputs.steerMotorConnected[i]);
      deviceDisconnectAlerts[i * 3 + 2].set(inputs.encoderConnected[i]);
    }

    Logger.recordOutput("Swerve/Pose", io.getPose());
  }

  private final double maxLinearSpeedMetersPerSecond =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularSpeedRadiansPerSecond =
      maxLinearSpeedMetersPerSecond
          / Math.max(
              Math.max(
                  Math.hypot(
                      TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                  Math.hypot(
                      TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
              Math.max(
                  Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                  Math.hypot(
                      TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();

  public Command teleopDrive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return run(
        () ->
            io.setControl(
                fieldCentricDrive
                    .withVelocityX(vx.getAsDouble() * maxLinearSpeedMetersPerSecond)
                    .withVelocityY(vy.getAsDouble() * maxLinearSpeedMetersPerSecond)
                    .withRotationalRate(omega.getAsDouble() * maxAngularSpeedRadiansPerSecond)));
  }

  public Command setToBlue() {
    return Commands.runOnce(() -> io.setOperatorPerspectiveForward(Rotation2d.kZero));
  }

  public Command setToRed() {
    return Commands.runOnce(() -> io.setOperatorPerspectiveForward(Rotation2d.k180deg));
  }
}
