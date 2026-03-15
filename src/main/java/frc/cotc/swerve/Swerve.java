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
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants;
import frc.cotc.FieldConstants;
import frc.cotc.FieldConstants.LeftBump;
import frc.cotc.FieldConstants.RightBump;
import frc.cotc.Robot;
import frc.cotc.shooter.Shooter;
import frc.cotc.vision.AprilTagPoseEstimator;
import frc.cotc.vision.AprilTagPoseEstimatorIOPhoton;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();
  private final PIDController pathXController = new PIDController(10, 0, 0);
  private final PIDController pathYController = new PIDController(10, 0, 0);
  private final PIDController pathThetaController = new PIDController(7, 0, 0);
  private final PIDController bumpAlignYController = new PIDController(10, 0, 1); // Placeholder
  private final PIDController bumpAlignThetaController = new PIDController(10, 0, 1); // Placeholder

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
    bumpAlignThetaController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
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
      deviceDisconnectAlerts[i * 3].set(!inputs.driveMotorConnected[i]);
      deviceDisconnectAlerts[i * 3 + 1].set(!inputs.steerMotorConnected[i]);
      deviceDisconnectAlerts[i * 3 + 2].set(!inputs.encoderConnected[i]);
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

  private final double slowModeMultiplier = 0.25;
  private double speedMultiplier = 1;

  public Command slowTeleopDrive() {
    return Commands.startEnd(() -> speedMultiplier = slowModeMultiplier, () -> speedMultiplier = 1);
  }

  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();

  public Command teleopDrive(Supplier<Translation2d> translationalInput, DoubleSupplier omega) {
    return run(() -> {
          var translation =
              Robot.isOnRed() ? translationalInput.get().unaryMinus() : translationalInput.get();
          var x = translation.getX();
          var y = translation.getY();
          io.setControl(
              fieldCentricDrive
                  .withVelocityX(x * speedMultiplier * maxLinearSpeedMetersPerSecond)
                  .withVelocityY(y * speedMultiplier * maxLinearSpeedMetersPerSecond)
                  .withRotationalRate(
                      omega.getAsDouble() * speedMultiplier * maxAngularSpeedRadiansPerSecond));
        })
        .withName("Teleop Drive");
  }

  private final SwerveRequest.FieldCentricFacingAngle facingAngle =
      new SwerveRequest.FieldCentricFacingAngle().withHeadingPID(8, 0, 0);

  private final PIDController distanceController = new PIDController(5, 0, 0);

  public Command aimAtTarget(
      Supplier<Translation2d> translationalInput, Shooter.ShotTarget target) {
    return run(() -> {
          var translational =
              Robot.isOnRed() ? translationalInput.get().unaryMinus() : translationalInput.get();
          var x = translational.getX();
          var y = translational.getY();

          var currentPoseToGoal =
              target
                  .getBaseTargetLocation()
                  .minus(getPose().plus(Constants.robotToShooterTransform).getTranslation());
          var currentPoseToGoalAngle = currentPoseToGoal.getAngle();
          var distanceToGoalMeters = currentPoseToGoal.getNorm();
          var distanceControllerOutput = distanceController.calculate(distanceToGoalMeters, 2.3);
          io.setControl(
              facingAngle
                  .withVelocityX(-distanceControllerOutput * currentPoseToGoalAngle.getCos() + x)
                  .withVelocityY(-distanceControllerOutput * currentPoseToGoalAngle.getSin() + y)
                  .withTargetDirection(
                      (target
                              .getWiggledTargetLocation()
                              .minus(
                                  getPose()
                                      .plus(Constants.robotToShooterTransform)
                                      .getTranslation())
                              .getAngle())
                          .minus(Constants.robotToShooterTransform.getRotation()))
                  .withTargetRateFeedforward(
                      (currentPoseToGoalAngle.getCos() * y + currentPoseToGoalAngle.getSin() * x)
                          / distanceToGoalMeters));
          Logger.recordOutput(
              "Shooter/Target", new Pose2d(target.getWiggledTargetLocation(), Rotation2d.kZero));
        })
        .withName("Aim at target");
  }

  public Command pass(Supplier<Translation2d> translationalInput) {
    return run(() -> {
          var translational =
              Robot.isOnRed() ? translationalInput.get().unaryMinus() : translationalInput.get();
          var x = translational.getX();
          var y = translational.getY();
          io.setControl(
              facingAngle
                  .withVelocityX(x * speedMultiplier * maxLinearSpeedMetersPerSecond)
                  .withVelocityY(y * speedMultiplier * maxLinearSpeedMetersPerSecond)
                  .withTargetDirection(
                      Robot.isOnRed()
                          ? Constants.robotToShooterTransform.getRotation().unaryMinus()
                          : Rotation2d.k180deg.minus(
                              Constants.robotToShooterTransform.getRotation())));
        })
        .withName("Pass");
  }

  public Command alignToBump(DoubleSupplier vx) {
    return teleopDrive(
        () -> {
          var bottomTrenchY = 2.5;
          var topTrenchY = FieldConstants.fieldWidth - bottomTrenchY;
          double targetY;
          if (getPose().getY() > FieldConstants.fieldWidth / 2) {
            targetY = topTrenchY;
          } else {
            targetY = bottomTrenchY;
          }
          return new Translation2d(
              vx.getAsDouble() * maxLinearSpeedMetersPerSecond,
              bumpAlignYController.calculate(getPose().getY(), targetY));
        },
        () -> bumpAlignThetaController.calculate(getPose().getRotation().getRadians(), 0));
  }

  public Command setToBlue() {
    return Commands.runOnce(() -> io.setOperatorPerspectiveForward(Rotation2d.kZero))
        .withName("Set to blue");
  }

  public Command setToRed() {
    return Commands.runOnce(() -> io.setOperatorPerspectiveForward(Rotation2d.k180deg))
        .withName("Set to red");
  }

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public Command brake() {
    return run(() -> io.setControl(brake)).withName("Brake");
  }

  public void followPath(SwerveSample sample) {
    var pose = getPose();

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    Logger.recordOutput("Swerve/Choreo target", sample.getPose());
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

  private Translation2d getProjectedPose(double futureSeconds) {
    Pose2d currentPose2d = getPose();

    ChassisSpeeds fieldRelativeSpeeds = getFieldSpeeds();

    return new Translation2d(
        currentPose2d.getX() + fieldRelativeSpeeds.vxMetersPerSecond * futureSeconds,
        currentPose2d.getY() + fieldRelativeSpeeds.vyMetersPerSecond * futureSeconds);
  }

  private final Rectangle2d alliLeftBump =
      new Rectangle2d(LeftBump.farRightCorner, LeftBump.nearLeftCorner);
  private final Rectangle2d oppLeftBump =
      new Rectangle2d(LeftBump.oppFarRightCorner, LeftBump.oppNearLeftCorner);
  private final Rectangle2d alliRightBump =
      new Rectangle2d(RightBump.farRightCorner, RightBump.nearLeftCorner);
  private final Rectangle2d oppRightBump =
      new Rectangle2d(RightBump.oppFarRightCorner, RightBump.oppNearLeftCorner);

  private int samples = 5;

  public boolean trajectoryWithinBump() {
    Translation2d projectedPose = getProjectedPose(0.1); // placeholder time
    Translation2d currentPose = getPose().getTranslation();

    // check on 5 projected points
    for (int i = 0; i <= samples; i++) {
      Translation2d projectedPoint = currentPose.interpolate(projectedPose, (double) i / samples);
      if (alliLeftBump.contains(projectedPoint)
          || oppLeftBump.contains(projectedPoint)
          || alliRightBump.contains(projectedPoint)
          || oppRightBump.contains(projectedPoint)) {
        return true;
      }
    }
    return false;
  }
}
