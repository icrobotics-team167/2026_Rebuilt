// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import frc.cotc.Robot;
import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveIOReal extends TunerConstants.TunerSwerveDrivetrain implements SwerveIO {
  private final ReentrantLock m_queueLock = new ReentrantLock();
  /* double buffer setup */
  private ArrayList<SwerveDriveState> stateQueue = new ArrayList<>();
  private ArrayList<SwerveDriveState> tmpStateQueue = new ArrayList<>();

  private Pose2d pose = new Pose2d();

  public SwerveIOReal(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);

    stateQueue.add(getStateCopy());
    registerTelemetry(this::updateTelemetry);

    if (Robot.isSimulation()) {
      double freq = 250;
      new Notifier(() -> updateSimState(1.0 / freq, 12)).startPeriodic(1.0 / freq);
    }
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    final var stateQueue = this.stateQueue;
    try {
      m_queueLock.lock();
      /* swap buffers */
      this.stateQueue = tmpStateQueue;
      tmpStateQueue = stateQueue;
    } finally {
      m_queueLock.unlock();
    }

    if (stateQueue.isEmpty()) {
      stateQueue.add(getStateCopy());
    }

    /* grab queues of information needed for odometry */
    inputs.poseQueue = new Pose2d[stateQueue.size()];
    inputs.modulePositionsQueue = new SwerveModulePosition[stateQueue.size()][4];
    inputs.rawHeadingQueue = new Rotation2d[stateQueue.size()];
    inputs.timestampQueue = new double[stateQueue.size()];
    for (int i = 0; i < stateQueue.size(); ++i) {
      final var state = stateQueue.get(i);
      inputs.poseQueue[i] = state.Pose;
      inputs.modulePositionsQueue[i] = state.ModulePositions;
      inputs.rawHeadingQueue[i] = state.RawHeading;
      inputs.timestampQueue[i] = Utils.currentTimeToFPGATime(state.Timestamp);
    }

    if (!stateQueue.isEmpty()) {
      /* grab the newest drive state */
      final var state = stateQueue.get(stateQueue.size() - 1);

      inputs.Speeds = state.Speeds;
      inputs.ModuleStates = state.ModuleStates;
      inputs.ModuleTargets = state.ModuleTargets;
      inputs.ModulePositions = state.ModulePositions;
      inputs.RawHeading = state.RawHeading;
      inputs.Timestamp = Utils.currentTimeToFPGATime(state.Timestamp);
      inputs.OdometryPeriod = state.OdometryPeriod;
      inputs.SuccessfulDaqs = state.SuccessfulDaqs;
      inputs.FailedDaqs = state.FailedDaqs;

      pose = state.Pose;
    }

    stateQueue.clear();
  }

  @Override
  public Pose2d getPose() {
    return pose;
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
  }

  private void updateTelemetry(SwerveDriveState state) {
    try {
      m_queueLock.lock();
      stateQueue.add(state.clone());
    } finally {
      m_queueLock.unlock();
    }
  }
}
