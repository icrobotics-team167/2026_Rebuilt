// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.cotc.Robot;
import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;

/** Implementation for a real swerve drivetrain using Phoenix swerve. */
public class SwerveIOReal extends TunerConstants.TunerSwerveDrivetrain implements SwerveIO {
  private final ReentrantLock queueLock = new ReentrantLock();
  /* double buffer setup */
  private ArrayList<SwerveDriveState> stateQueue = new ArrayList<>();
  private ArrayList<SwerveDriveState> tmpStateQueue = new ArrayList<>();

  private Pose2d pose = new Pose2d();

  private final BaseStatusSignal[] connectedSignals;

  public SwerveIOReal() {
    this(
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
  }

  SwerveIOReal(SwerveModuleConstants<?, ?, ?>... modules) {
    super(TunerConstants.DrivetrainConstants, modules);

    stateQueue.add(getStateCopy());
    registerTelemetry(this::updateTelemetry);

    connectedSignals = new BaseStatusSignal[3 * 4];
    for (int i = 0; i < 4; i++) {
      connectedSignals[i * 3] = getModule(i).getDriveMotor().getVersion();
      connectedSignals[i * 3 + 1] = getModule(i).getSteerMotor().getVersion();
      connectedSignals[i * 3 + 2] = getModule(i).getEncoder().getVersion();
    }
    Robot.canivoreSignals.addSignals(connectedSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(10, connectedSignals);
  }

  private void updateTelemetry(SwerveDriveState state) {
    try {
      // Loctite™️
      queueLock.lock();
      // Add the latest state to the queue
      stateQueue.add(state.clone());
    } finally {
      queueLock.unlock();
    }
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    final var stateQueue = this.stateQueue;
    try {
      // Loctite™️
      queueLock.lock();
      // Swap buffers so that the other buffer can be filled while we process this one
      this.stateQueue = tmpStateQueue;
      tmpStateQueue = stateQueue;
    } finally {
      // Unlock
      queueLock.unlock();
    }

    // Grab queues of data needed for odometry
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
      // Grab the newsest state
      final var state = stateQueue.get(stateQueue.size() - 1);

      // Fill in the data with the latest state
      inputs.Speeds = state.Speeds;
      inputs.ModuleStates = state.ModuleStates;
      inputs.ModuleTargets = state.ModuleTargets;
      inputs.ModulePositions = state.ModulePositions;

      pose = state.Pose;
    }

    stateQueue.clear();

    for (int i = 0; i < 4; i++) {
      inputs.driveMotorConnected[i] = connectedSignals[i * 3].getStatus().isOK();
      inputs.steerMotorConnected[i] = connectedSignals[i * 3 + 1].getStatus().isOK();
      inputs.encoderConnected[i] = connectedSignals[i * 3 + 2].getStatus().isOK();
    }
    inputs.timeOffsetSeconds = Utils.fpgaToCurrentTime(0);
  }

  // updateOdometry is a noop in real/sim since we can use the real pose estimator directly

  @Override
  public Pose2d getPose() {
    return pose;
  }
}
