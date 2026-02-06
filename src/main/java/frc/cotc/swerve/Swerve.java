// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants;
import frc.cotc.Robot;
import frc.cotc.vision.AprilTagPoseEstimator;
import frc.cotc.vision.AprilTagPoseEstimatorIOPhoton;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();
  private final PIDController pathXController = new PIDController(10, 0, 0);
  private final PIDController pathYController = new PIDController(10, 0, 0);
  private final PIDController pathThetaController = new PIDController(7, 0, 0);

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
    pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private final ArrayList<Pose2d> visionPoses = new ArrayList<>();

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
    for (var camera : cameras) {
      camera.update(
          (Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) -> {
            visionPoses.add(pose);
            io.addVisionMeasurement(pose, timestamp + inputs.timeOffsetSeconds, stdDevs);
          },
          DriverStation.isEnabled() ? getPose() : null);
    }
    Logger.recordOutput("Swerve/Vision Poses", visionPoses.toArray(new Pose2d[0]));
    visionPoses.clear();

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
    return run(() ->
            io.setControl(
                fieldCentricDrive
                    .withVelocityX(vx.getAsDouble() * maxLinearSpeedMetersPerSecond)
                    .withVelocityY(vy.getAsDouble() * maxLinearSpeedMetersPerSecond)
                    .withRotationalRate(omega.getAsDouble() * maxAngularSpeedRadiansPerSecond)))
        .withName("Teleop Drive");
  }

  public Command setToBlue() {
    return Commands.runOnce(() -> io.setOperatorPerspectiveForward(Rotation2d.kZero))
        .withName("Set to blue");
  }

  public Command setToRed() {
    return Commands.runOnce(() -> io.setOperatorPerspectiveForward(Rotation2d.k180deg))
        .withName("Set to red");
  }

  public void followPath(SwerveSample sample) {
    var pose = getPose();

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    io.setControl(
        m_pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  public Pose2d getPose() {
    return io.getPose();
  }

  public ChassisSpeeds getRobotSpeeds() {
    return inputs.Speeds;
  }

  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(inputs.Speeds, getPose().getRotation());
  }

  public void resetPose(Pose2d pose) {
    if (io instanceof SwerveIOSim simImpl) {
      simImpl.resetPose(pose);
    }
    if (Robot.mode == Robot.Mode.SIM) {
      AprilTagPoseEstimatorIOPhoton.resetSim();
    }
    io.resetPose(pose);
  }
}
