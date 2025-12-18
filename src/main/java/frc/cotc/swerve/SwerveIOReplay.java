// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;

/** Replaces the Phoenix Swerve pose estimator with a replay-compatible one. */
public class SwerveIOReplay extends TunerConstants.TunerSwerveDrivetrain implements SwerveIO {
  private final SwerveDrivePoseEstimator poseEstimator;
  private boolean poseEstInit = false;

  public SwerveIOReplay() {
    super(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);

    // Initialize pose estimator
    // We create an array of module positions with zeroed positions to avoid a NullPointerException
    var modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }
    poseEstimator =
        new SwerveDrivePoseEstimator(
            getKinematics(), Rotation2d.kZero, modulePositions, Pose2d.kZero);
  }

  @Override
  public void updateOdometry(SwerveIOInputs inputs) {
    // Update the shimmed pose estimator with the odometry updates captured from the inputs

    // In replay, the pose estimator is constructed with zeroed everything, but in hardware that's
    // not the case, Phoenix swerve constructs it with non-zero heading and module positions.
    // The resetPosition() is functioning as the initial constructor with the initial measurements.
    // Can't explicitly pass in a Pose2d.kZero in the unlikely event that the hardware one
    // already read movement associated with those measurements.
    // - Ben CTRE (Paraphrased)
    if (!poseEstInit) {
      poseEstimator.resetPosition(
          inputs.rawHeadingQueue[0], inputs.modulePositionsQueue[0], inputs.poseQueue[0]);
      poseEstInit = true;
    }

    // Apply all the logged odometry updates
    for (int i = 0; i < inputs.timestampQueue.length; ++i) {
      // Apply update
      poseEstimator.updateWithTime(
          inputs.timestampQueue[i], inputs.rawHeadingQueue[i], inputs.modulePositionsQueue[i]);
    }
  }

  // *** Shims over the Phoenix swerve pose estimator ***

  @Override
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void tareEverything() {
    resetPose(Pose2d.kZero);
  }

  @Override
  public void seedFieldCentric() {
    poseEstimator.resetRotation(getOperatorForwardDirection());
  }

  @Override
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  @Override
  public void resetTranslation(Translation2d translation) {
    poseEstimator.resetTranslation(translation);
  }

  @Override
  public void resetRotation(Rotation2d rotation) {
    poseEstimator.resetRotation(rotation);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return poseEstimator.sampleAt(timestampSeconds);
  }
}
