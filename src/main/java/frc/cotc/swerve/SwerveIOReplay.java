// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;

public class SwerveIOReplay extends TunerConstants.TunerSwerveDrivetrain implements SwerveIO {
  private final SwerveDrivePoseEstimator poseEstimator;
  private boolean poseEstInit = false;

  private Pose2d m_pose = new Pose2d();
  private SwerveModuleState[] moduleTargets;

  public SwerveIOReplay(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);

    final var modulePositions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modulePositions.length; ++i) {
      modulePositions[i] = new SwerveModulePosition();
    }
    poseEstimator =
        new SwerveDrivePoseEstimator(
            getKinematics(), Rotation2d.kZero, modulePositions, Pose2d.kZero);
    moduleTargets = new SwerveModuleState[modules.length];
    for (int i = 0; i < moduleTargets.length; ++i) {
      moduleTargets[i] = new SwerveModuleState();
    }
  }

  @Override
  public void updateOdometry(SwerveIOInputs inputs) {
    if (!poseEstInit) {
      poseEstimator.resetPosition(
          inputs.rawHeadingQueue[0], inputs.modulePositionsQueue[0], inputs.poseQueue[0]);
      poseEstInit = true;
    }

    for (int i = 0; i < inputs.timestampQueue.length; ++i) {
      // Apply update
      poseEstimator.updateWithTime(
          inputs.timestampQueue[i], inputs.rawHeadingQueue[i], inputs.modulePositionsQueue[i]);
    }

    m_pose = poseEstimator.getEstimatedPosition();
    // TODO: force a control refresh and pull out the new module targets
    moduleTargets = inputs.ModuleTargets;
  }

  @Override
  public Pose2d getPose() {
    return m_pose;
  }

  @Override
  public SwerveModuleState[] getModuleTargets() {
    return moduleTargets;
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
  public void seedFieldCentric(Rotation2d rotation) {
    poseEstimator.resetRotation(rotation.plus(getOperatorForwardDirection()));
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
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
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
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }

  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return poseEstimator.sampleAt(timestampSeconds);
  }
}
