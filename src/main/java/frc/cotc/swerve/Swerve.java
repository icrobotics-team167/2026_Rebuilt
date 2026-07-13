// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants;
import frc.cotc.FieldConstants;
import frc.cotc.FieldConstants.LeftBump;
import frc.cotc.FieldConstants.RightBump;
import frc.cotc.Robot;
import frc.cotc.shooter.SOTM;
import frc.cotc.vision.AprilTagPoseEstimator;
import frc.cotc.vision.AprilTagPoseEstimator.VisionMeasurement;
import frc.cotc.vision.AprilTagPoseEstimatorIOPhoton;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  // Choreo path following request
  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
  // PID controllers for Choreo path following
  // I never ended up tuning this lmao. Tuning the feedforwards and the paths themselves worked
  // Well Enough™
  private final PIDController pathXController = new PIDController(10, 0, 0);
  private final PIDController pathYController = new PIDController(10, 0, 0);
  private final PIDController pathThetaController = new PIDController(7, 0, 0);
  // PID controllers for automatic alignment to the bump
  // We never ended up using automatic bump align, since Edmund just got good at doing it manually.
  private final PIDController bumpAlignYController = new PIDController(10, 0, 1); // Placeholder
  private final PIDController bumpAlignThetaController = new PIDController(8, 0, 1); // Placeholder

  // Alerts to warn of disconnects in the drivetrain.
  // I probably should've bothered for the other subsystems too, but I was feeling lazy, and the
  // drivetrain is the highest priority subsystem, and we've historically had wiring issues with
  // the drivetrain. Saved our butts a few times this year.
  private final Alert[] deviceDisconnectAlerts = new Alert[12];

  // Dunno why this is package-protected and not private final.
  AprilTagPoseEstimator[] cameras;

  @SuppressWarnings("resource")
  public Swerve(SwerveIO io, AprilTagPoseEstimator... cameras) {
    this.io = io;

    this.cameras = cameras;
    // HACK: Share one Consumer object across the multiple cameras to minimize objects in memory
    // (We only got 512mb and most of it is taken up by the OS!)
    // Actually, this is probably an imperfect solution, since we're still creating a new
    // Consumer object. I could've saved more memory by passing measurements and visionPoses into
    // the camera objects at the .update() method. But that's also ugly and also a tiny
    // optimization compared to the bigger optimizations I could've done if the need arose.
    Consumer<VisionMeasurement> measurementConsumer =
        measurement -> {
          measurements.add(measurement);
          visionPoses.add(measurement.pose());
        };
    for (var camera : cameras) {
      camera.setEstimateConsumer(measurementConsumer);
    }

    // Capture initial state and set up odometry
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    io.updateOdometry(inputs);

    // Set up alerts for motor disconnects
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
    // Set up PID controller wrapping
    pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
    // Wrapping goes +π/2 to -π/2 so that 0 and ±π are considered the same
    // This allows for the bump align controller to align to *either* 0 or 180 degrees, both of
    // which are valid for alignment to the bump.
    bumpAlignThetaController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
  }

  /** ArrayList to store processed vision measurements for logging. */
  private final ArrayList<Pose2d> visionPoses = new ArrayList<>();

  /** Comparator to sort measurements by timestamp. */
  private final Comparator<VisionMeasurement> measurementComparator =
      Comparator.comparingDouble(VisionMeasurement::timestamp);

  /** ArrayList to store vision measurements for processing. */
  private final ArrayList<VisionMeasurement> measurements = new ArrayList<>();

  @Override
  public void periodic() {
    // Update and process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    // Update odometry using those inputs
    io.updateOdometry(inputs);

    if (Robot.mode == Robot.Mode.SIM) {
      // If sim, update the vision sim
      AprilTagPoseEstimatorIOPhoton.updateSim();
    }
    // Clear measurements from the last cycle
    measurements.clear();
    // Loop over the cameras to update them
    for (var camera : cameras) {
      camera.addPoseData(Timer.getTimestamp(), getPose());
      camera.update();
    }
    // If the measurements are not chronologically sorted, the pose estimator discards out-of-
    // order measurements. To avoid this, we sort using the timestamp.
    measurements.sort(measurementComparator);
    // Loop over the measurements and add them to the estimator.
    for (var measurement : measurements) {
      io.addVisionMeasurement(
          measurement.pose(),
          measurement.timestamp() + inputs.timeOffsetSeconds,
          measurement.stdDevs());
    }
    // Log the vision measurements
    Logger.recordOutput("Swerve/Vision Poses", visionPoses.toArray(new Pose2d[0]));
    visionPoses.clear();

    // Update disconnect alerts
    // If the device disconnects, the alert will be set to true, and the alert will be displayed on
    // the dashboard.
    for (int i = 0; i < 4; i++) {
      deviceDisconnectAlerts[i * 3].set(!inputs.driveMotorConnected[i]);
      deviceDisconnectAlerts[i * 3 + 1].set(!inputs.steerMotorConnected[i]);
      deviceDisconnectAlerts[i * 3 + 2].set(!inputs.encoderConnected[i]);
    }

    // Log the final pose estimate
    Logger.recordOutput("Swerve/Pose", io.getPose());
  }

  private final double maxLinearSpeedMetersPerSecond =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularSpeedRadiansPerSecond =
      // Max linear speed / distance of farthest wheel = max angular speed
      maxLinearSpeedMetersPerSecond
          /
          // Giant math.max to find the farthest wheel
          // Technically not necessary since we used a centered rectangle as our drivetrain size
          // and therefore all 4 of the wheels were the same distance
          Math.max(
              Math.max(
                  Math.hypot(
                      TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                  Math.hypot(
                      TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
              Math.max(
                  Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                  Math.hypot(
                      TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // Going full speed by default was too fast for Edmund, so we default to 80% speed
  // Edmund also requested a slow mode for the drivetrain, so 33% is used when driving in slow mode.
  private final double slowModeMultiplier = 0.33;
  private final double baseSpeedMultiplier = 0.8;
  private double speedMultiplier = baseSpeedMultiplier;

  // We use Commands.startEnd instead of this.startEnd because this.startEnd requires this
  // subsystem, but we don't want this command to require this subsystem, because it would prevent
  // other commands from using this subsystem.
  public Command slowTeleopDrive() {
    return Commands.startEnd(
        () -> speedMultiplier = slowModeMultiplier, () -> speedMultiplier = baseSpeedMultiplier);
  }

  public Command fastTeleopDrive() {
    return Commands.startEnd(
        () -> speedMultiplier = 1, () -> speedMultiplier = baseSpeedMultiplier);
  }

  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public Command teleopDrive(Supplier<Translation2d> translationalInput, DoubleSupplier omega) {
    return run(() -> {
          // If the robot is on red, the driver-relative inputs will be flipped compared to the
          // absolute field-relative controls, since the field-relative controls are defined using
          // the blue alliance. To fix this, we flip the controls 180 degrees. The most efficient
          // way to do this is to just invert the vector.
          var translation =
              Robot.isOnRed() ? translationalInput.get().unaryMinus() : translationalInput.get();
          var x = translation.getX();
          var y = translation.getY();
          io.setControl(
              fieldCentricDrive
                  .withVelocityX(x * speedMultiplier * maxLinearSpeedMetersPerSecond)
                  .withVelocityY(y * speedMultiplier * maxLinearSpeedMetersPerSecond)
                  .withRotationalRate(
                      omega.getAsDouble()
                          * speedMultiplier
                          * maxAngularSpeedRadiansPerSecond
                          * 0.5)); // HACK: Edmund found the default rotation speed too fast
        })
        .withName("Teleop Drive");
  }

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withHeadingPID(6, 0, 0.1)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private Rotation2d lastHeading = Rotation2d.kZero;

  /**
   * Mode to point the right stick to aim the direction instead of using the right stick to move it
   * manually.
   *
   * <p>Edmund requested this to automate aiming, but the imprecision of aiming with a stick
   * direction and bad sightlines in some parts of the field made this not worth. Maybe would've
   * worked if we had a BattleBots Orbitron style overhead camera view to make it more like Hotline
   * Miami, but driver station rules banned that after many teams did Shenanigans™ with smth similar
   * in 2016.
   */
  public Command faceAngle(
      Supplier<Translation2d> translationalInput, Supplier<Translation2d> headingInput) {
    return run(
        () -> {
          var translation =
              Robot.isOnRed() ? translationalInput.get().unaryMinus() : translationalInput.get();
          var x = translation.getX();
          var y = translation.getY();

          // Same thing as inverting the translational input on red.
          var headingControl =
              Robot.isOnRed() ? headingInput.get().unaryMinus() : headingInput.get();
          Rotation2d heading;
          // If the control input is close to zero, just fall back to the last inputted heading
          // to avoid precision/divide-by-zero errors.
          if (headingControl.getNorm() < 1e-3) {
            heading = lastHeading;
          } else {
            heading = headingControl.getAngle();
            lastHeading = heading;
          }
          io.setControl(
              fieldCentricFacingAngle
                  .withVelocityX(x * maxLinearSpeedMetersPerSecond)
                  .withVelocityY(y * maxLinearSpeedMetersPerSecond)
                  .withTargetDirection(heading));
        });
  }

  private final SwerveRequest.FieldCentricFacingAngle shootingAim =
      new SwerveRequest.FieldCentricFacingAngle()
          .withHeadingPID(12, 0, 0)
          .withCenterOfRotation(Constants.robotToShooterTransform.getTranslation())
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private SOTM.SOTMResult sotmResult;

  // HACK: The SOTM result needs to be shared between the otherwise entirely separated swerve and
  // shooter subsystems, requiring some method of transferring data. This is it.
  public void setSOTMResult(SOTM.SOTMResult result) {
    this.sotmResult = result;
  }

  public Command aimAtTarget(Supplier<Translation2d> translationalInput) {
    return run(() -> {
          // Avoid a NPE
          if (sotmResult == null) return;
          var translational =
              Robot.isOnRed() ? translationalInput.get().unaryMinus() : translationalInput.get();
          Logger.recordOutput(
              "Swerve/Aiming target yaw",
              sotmResult.yaw().minus(Constants.robotToShooterTransform.getRotation()));

          var currentPoseToGoal =
              Robot.shotTarget.targetLocation.minus(
                  getPose().plus(Constants.robotToShooterTransform).getTranslation());
          var currentPoseToGoalAngle = currentPoseToGoal.getAngle();
          var distanceToGoalMeters = currentPoseToGoal.getNorm();

          // Clamp velocity away from the goal to -0.25m/s to avoid running away from the target
          // too fast
          // Can't shoot into the goal if we're outrunning our own projectile.
          var targetRelativeSpeed = translational.rotateBy(currentPoseToGoalAngle.unaryMinus());
          if (targetRelativeSpeed.getX() < -0.25) {
            translational = translational.times(-0.25 / targetRelativeSpeed.getX());
          }
          var x = translational.getX();
          var y = translational.getY();
          io.setControl(
              shootingAim
                  .withVelocityX(
                      x
                          // Clamp max move speed to the max speed defined by the SOTM calculation
                          // result.
                          * Math.min(
                              maxLinearSpeedMetersPerSecond,
                              sotmResult.maxMoveSpeedMetersPerSecond()))
                  .withVelocityY(
                      y
                          * Math.min(
                              maxLinearSpeedMetersPerSecond,
                              sotmResult.maxMoveSpeedMetersPerSecond()))
                  .withTargetDirection(
                      sotmResult.yaw().minus(Constants.robotToShooterTransform.getRotation()))
                  .withTargetRateFeedforward(
                      // Feedforward to keep the swerve pointed at the target
                      // HACK: This feedforward uses the velocity needed to stay pointed at the
                      // stationary real target, but in actuality we are aiming at the constantly
                      // moving virtual target. It *seems* to be close enough that it's better than
                      // no FF.
                      (currentPoseToGoalAngle.getCos() * y + currentPoseToGoalAngle.getSin() * x)
                          / distanceToGoalMeters));
        })
        .withName("Aim at target");
  }

  /**
   * Originally used to align to the trench, this command was repurposed to align to the bump. We
   * never ended up using it though since we couldn't get it to feel right in the controls and
   * Edmund got good at aligning manually anyways.
   */
  public Command alignToBump(DoubleSupplier vx) {
    return teleopDrive(
        () -> {
          // This code was written by Mert before I realized that FieldConstants had a defined
          // y-coordinate for the trenches and bumps.
          // The name is also inaccurate since we never renamed after we realized the first
          // iteration of the robot don't fit under the trench.
          var bottomTrenchY = 2.5;
          var topTrenchY = FieldConstants.fieldWidth - bottomTrenchY;
          double targetY;
          // If on the other side of the field, flip the target y-coordinate to go to the closer
          // trench/bump.
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

  /**
   * Unlike PathPlanner that does most of the path following logic itself, Choreo gives that task to
   * the user. Gives more flexibility that way.
   */
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
            // Feedforwards for the torque outputs to the motors.
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  public Command pidToPose(Pose2d targetPose) {
    return run(
        () -> {
          var currentPose = getPose();

          io.setControl(
              m_pathApplyFieldSpeeds
                  .withSpeeds(
                      new ChassisSpeeds(
                          pathXController.calculate(currentPose.getX(), targetPose.getX()),
                          pathYController.calculate(currentPose.getY(), targetPose.getY()),
                          pathThetaController.calculate(
                              currentPose.getRotation().getRadians(),
                              targetPose.getRotation().getRadians())))
                  .withWheelForceFeedforwardsX(new double[4])
                  .withWheelForceFeedforwardsY(new double[4]));
        });
  }

  public Pose2d getPose() {
    return io.getPose();
  }

  /**
   * Robot-relative velocity of the drivetrain.
   */
  public ChassisSpeeds getRobotSpeeds() {
    return inputs.Speeds;
  }

  /**
   * Field-relative velocity of the drivetrain.
   */
  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(inputs.Speeds, getPose().getRotation());
  }

  /**
   * Reset the current odometry position.
   */
  public void resetPose(Pose2d pose) {
    // Hrm. Why did I do this.
    // I think it was because the vision sim reset (see below) wasn't correctly resetting, so I
    // added a resetPose call above that, thinking the sim reset needed extra handling, but the
    // root cause of the bug was that the vision system was being reset before the swerve sim was
    // reset in the resetPose call at the bottom.
    if (io instanceof SwerveIOSim simImpl) {
      simImpl.resetPose(pose);
    }
    // Reset the vision system sim's internal state
    if (Robot.mode == Robot.Mode.SIM) {
      AprilTagPoseEstimatorIOPhoton.resetSim();
    }
    // Reset the pose estimator's pose.
    io.resetPose(pose);
  }

  private Translation2d getProjectedPose(
      double futureSeconds, Supplier<Translation2d> translationalInput) {
    Pose2d currentPose2d = getPose();

    var translation =
        Robot.isOnRed() ? translationalInput.get().unaryMinus() : translationalInput.get();
    var x = translation.getX();
    var y = translation.getY();

    return new Translation2d(
        currentPose2d.getX() + x * maxLinearSpeedMetersPerSecond * futureSeconds,
        currentPose2d.getY() + y * maxLinearSpeedMetersPerSecond * futureSeconds);
  }

  private final Rectangle2d alliLeftBump =
      new Rectangle2d(LeftBump.farRightCorner, LeftBump.nearLeftCorner);
  private final Rectangle2d oppLeftBump =
      new Rectangle2d(LeftBump.oppFarRightCorner, LeftBump.oppNearLeftCorner);
  private final Rectangle2d alliRightBump =
      new Rectangle2d(RightBump.farRightCorner, RightBump.nearLeftCorner);
  private final Rectangle2d oppRightBump =
      new Rectangle2d(RightBump.oppFarRightCorner, RightBump.oppNearLeftCorner);

  private final int samples = 5;

  /**
   * Was originally intended to detect if the future trajectory of the robot, assuming a
   * straight-line constant-velocity trajectory, intersects the bump, and therefore we
   * should automatically align to the bump. We never ended up using this since this was for the
   * bump autoalign which we never ended up using.
   */
  public boolean trajectoryWithinBump(Supplier<Translation2d> translationalInput) {
    Logger.recordOutput(
        "Swerve/Bumps/Alli Left",
        new Pose2d(LeftBump.farRightCorner, Rotation2d.kZero),
        new Pose2d(LeftBump.nearLeftCorner, Rotation2d.kZero));
    Logger.recordOutput(
        "Swerve/Bumps/Opp Left",
        new Pose2d(LeftBump.oppFarRightCorner, Rotation2d.kZero),
        new Pose2d(LeftBump.oppNearLeftCorner, Rotation2d.kZero));
    Logger.recordOutput(
        "Swerve/Bumps/Alli Right",
        new Pose2d(RightBump.farRightCorner, Rotation2d.kZero),
        new Pose2d(RightBump.nearLeftCorner, Rotation2d.kZero));
    Logger.recordOutput(
        "Swerve/Bumps/Opp Right",
        new Pose2d(RightBump.oppFarRightCorner, Rotation2d.kZero),
        new Pose2d(RightBump.oppNearLeftCorner, Rotation2d.kZero));

    var currentPose = getPose().getTranslation();
    var projectedPose = getProjectedPose(0.5, translationalInput);

    // HACK: A proper collision check between an axis-aligned bounding box and a line segment was
    // too much work, so we took the Mario 64 approach of doing the collision checks at discrete
    // sample points.
    var projectedPoses = new ArrayList<Pose2d>();
    // check on 5 projected points
    for (int i = 0; i <= samples; i++) {
      Translation2d projectedPoint = currentPose.interpolate(projectedPose, (double) i / samples);
      projectedPoses.add(new Pose2d(projectedPoint, getPose().getRotation()));
      if (alliLeftBump.contains(projectedPoint)
          || oppLeftBump.contains(projectedPoint)
          || alliRightBump.contains(projectedPoint)
          || oppRightBump.contains(projectedPoint)) {
        Logger.recordOutput("Swerve/Bump Align Detections", projectedPoses.toArray(new Pose2d[0]));
        return true;
      }
    }
    Logger.recordOutput("Swerve/Bump Align Detections", projectedPoses.toArray(new Pose2d[0]));
    return false;
  }
}
