// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  class SwerveIOInputs {
    /* latest swerve drivetrain state */
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    public SwerveModuleState[] ModuleStates = new SwerveModuleState[4];
    public SwerveModuleState[] ModuleTargets = new SwerveModuleState[4];
    public SwerveModulePosition[] ModulePositions = new SwerveModulePosition[4];
    public Rotation2d RawHeading = new Rotation2d();
    public double Timestamp = 0.0;
    public double OdometryPeriod = 0.0;
    public int SuccessfulDaqs = 0;
    public int FailedDaqs = 0;

    /* queues for odometry updates */
    public Pose2d[] poseQueue = new Pose2d[0];
    public SwerveModulePosition[][] modulePositionsQueue = new SwerveModulePosition[4][0];
    public Rotation2d[] rawHeadingQueue = new Rotation2d[0];
    public double[] timestampQueue = new double[0];
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(SwerveIOInputs inputs) {}

  /** Updates the odometry pose estimate. */
  default void updateOdometry(SwerveIOInputs inputs) {}

  /** Gets the robot pose estimate. */
  Pose2d getPose();

  default void clearVisionMeasurements() {}

  /**
   * Zero's this swerve drive's odometry entirely.
   *
   * <p>This will zero the entire odometry, and place the robot at 0,0
   */
  void tareEverything();

  /**
   * Resets the rotation of the robot pose to 0 from the {@link
   * SwerveRequest.ForwardPerspectiveValue#OperatorPerspective} perspective.
   *
   * <p>This is equivalent to calling {@link #resetRotation} with the operator perspective rotation.
   */
  void seedFieldCentric();

  /**
   * Resets the pose of the robot. The pose should be from the {@link
   * SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   */
  void resetPose(Pose2d pose);

  /**
   * Resets the translation of the robot pose without affecting rotation. The translation should be
   * from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   */
  void resetTranslation(Translation2d translation);

  /**
   * Resets the rotation of the robot pose without affecting translation. The rotation should be
   * from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   */
  void resetRotation(Rotation2d rotation);

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   */
  void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);

  void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);

  /** Sets the pose estimator's trust of global measurements. */
  void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs);

  /** Return the pose at a given timestamp, if the buffer is not empty. */
  Optional<Pose2d> samplePoseAt(double timestampSeconds);

  // ----------- Everything below is inherited from SwerveDrivetrain -----------//

  /** Gets whether the drivetrain is on a CAN FD bus. */
  boolean isOnCANFD();

  /** Gets a reference to the kinematics used for the drivetrain. */
  SwerveDriveKinematics getKinematics();

  /** Applies the specified control request to the swerve drivetrain. */
  void setControl(SwerveRequest request);

  /** Configures the neutral mode to use for all modules' drive motors. */
  StatusCode configNeutralMode(NeutralModeValue neutralMode);

  /**
   * Takes the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perpective direction and
   * treats it as the forward direction for {@link
   * SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}.
   */
  void setOperatorPerspectiveForward(Rotation2d fieldDirection);

  /**
   * Returns the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perpective direction
   * that is treated as the forward direction for {@link
   * SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}.
   */
  Rotation2d getOperatorForwardDirection();

  /** Gets the locations of the swerve modules. */
  Translation2d[] getModuleLocations();
}
