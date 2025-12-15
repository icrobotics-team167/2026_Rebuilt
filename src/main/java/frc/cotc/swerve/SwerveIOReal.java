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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.cotc.Robot;
import java.util.ArrayList;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveIOReal extends TunerConstants.TunerSwerveDrivetrain implements SwerveIO {
  private final ReentrantLock m_queueLock = new ReentrantLock();
  /* double buffer setup */
  private ArrayList<SwerveDriveState> m_stateQueue = new ArrayList<>();
  private ArrayList<SwerveDriveState> m_tmpStateQueue = new ArrayList<>();

  private Pose2d m_pose = new Pose2d();
  private SwerveModuleState[] m_moduleTargets;

  public SwerveIOReal(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);

    final var initialState = getStateCopy();
    m_stateQueue.add(initialState);
    m_moduleTargets = initialState.ModuleTargets;

    registerTelemetry(this::updateTelemetry);

    if (Robot.isSimulation()) {
      double freq = 250;
      new Notifier(() -> updateSimState(1.0 / freq, 12)).startPeriodic(1.0 / freq);
    }
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    final var stateQueue = m_stateQueue;
    try {
      m_queueLock.lock();
      /* swap buffers */
      m_stateQueue = m_tmpStateQueue;
      m_tmpStateQueue = stateQueue;
    } finally {
      m_queueLock.unlock();
    }

    inputs.fpgaToCurrentTime = Utils.getCurrentTimeSeconds() - Timer.getFPGATimestamp();

    /* grab queues of information needed for odometry */
    inputs.poseQueue = new Pose2d[stateQueue.size()];
    inputs.modulePositionsQueue = new SwerveModulePosition[stateQueue.size()][];
    inputs.rawHeadingQueue = new Rotation2d[stateQueue.size()];
    inputs.timestampQueue = new double[stateQueue.size()];
    for (int i = 0; i < stateQueue.size(); ++i) {
      final var state = stateQueue.get(i);
      inputs.poseQueue[i] = state.Pose;
      inputs.modulePositionsQueue[i] = state.ModulePositions;
      inputs.rawHeadingQueue[i] = state.RawHeading;
      inputs.timestampQueue[i] = state.Timestamp;
    }

    if (!stateQueue.isEmpty()) {
      /* grab the newest drive state */
      final var state = stateQueue.get(stateQueue.size() - 1);

      inputs.Speeds = state.Speeds;
      inputs.ModuleStates = state.ModuleStates;
      inputs.ModuleTargets = state.ModuleTargets;
      inputs.ModulePositions = state.ModulePositions;
      inputs.RawHeading = state.RawHeading;
      inputs.Timestamp = state.Timestamp;
      inputs.OdometryPeriod = state.OdometryPeriod;
      inputs.SuccessfulDaqs = state.SuccessfulDaqs;
      inputs.FailedDaqs = state.FailedDaqs;

      m_pose = state.Pose;
      m_moduleTargets = state.ModuleTargets;
    }

    stateQueue.clear();
  }

  @Override
  public Pose2d getPose() {
    return m_pose;
  }

  @Override
  public SwerveModuleState[] getModuleTargets() {
    return m_moduleTargets;
  }

  private void updateTelemetry(SwerveDriveState state) {
    try {
      m_queueLock.lock();
      m_stateQueue.add(state.clone());
    } finally {
      m_queueLock.unlock();
    }
  }
}
