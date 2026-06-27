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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.CircularBuffer;
import frc.cotc.Robot;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Implementation for a real swerve drivetrain using Phoenix swerve.
 *
 * <p>This came about due to an idea that Ben Hall (CTRE Intern) had, where since Phoenix Swerve's
 * pose estimation just uses the WPILib pose estimator, and the Phoenis Swerve API exposes the
 * ability to get data from the internal odometry thread, it would be possible to log the data and
 * use it for AdvantageKit replay compatibility. This sidesteps the main problem of using a
 * black-box swerve library with AKit, where pose estimation is inside the black box and therefore
 * cannot be replayed.
 *
 * <p>This class is the component that logs the data to AdvantageKit.
 */
public class SwerveIOReal extends TunerConstants.TunerSwerveDrivetrain implements SwerveIO {
  private final ReentrantLock queueLock = new ReentrantLock();
  // Double buffer setup
  // "queue" is a misnomer from back when Ben's original impl used an ArrayDeque. I then changed
  // it to an ArrayList to use the more friendly API in that class.
  // Problem is, both ArrayDeques and ArrayLists are uhm. Unbounded.
  // "[@Jonah | 6328M | WPILib] I accidentally created a wpilog that turns ascope into a zip bomb"
  // - Me
  // The data loss and thus real-replay discrepancy from using a CircularBuffer with a limited
  // max size is well worth not exploding my computer.
  private CircularBuffer<SwerveDriveState> stateQueue = new CircularBuffer<>(50);
  private CircularBuffer<SwerveDriveState> tmpStateQueue = new CircularBuffer<>(50);

  private Pose2d pose = new Pose2d();

  private final BaseStatusSignal[] connectedSignals;

  private final BaseStatusSignal[] currentSignals = new BaseStatusSignal[16];

  public SwerveIOReal() {
    this(
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
  }

  SwerveIOReal(SwerveModuleConstants<?, ?, ?>... modules) {
    super(TunerConstants.DrivetrainConstants, modules);

    stateQueue.addLast(getStateCopy());
    // When the high-frequency odometry thread updates 250 times a second, updateTelemetry() gets
    // called.
    // Due to the odometry thread running 5x faster than the main robot code thread, this
    // requires some extra care to ensure thread safety
    registerTelemetry(this::updateTelemetry);

    connectedSignals = new BaseStatusSignal[3 * 4];
    for (int i = 0; i < 4; i++) {
      // Log whether the motor is connected or not by polling the version signal, which should
      // always be sent.
      connectedSignals[i * 3] = getModule(i).getDriveMotor().getVersion(false);
      connectedSignals[i * 3 + 1] = getModule(i).getSteerMotor().getVersion(false);
      connectedSignals[i * 3 + 2] = getModule(i).getEncoder().getVersion(false);

      currentSignals[i * 4] = getModule(i).getDriveMotor().getStatorCurrent(false);
      currentSignals[i * 4 + 1] = getModule(i).getDriveMotor().getSupplyCurrent(false);
      currentSignals[i * 4 + 2] = getModule(i).getSteerMotor().getStatorCurrent(false);
      currentSignals[i * 4 + 3] = getModule(i).getSteerMotor().getSupplyCurrent(false);
    }
    Robot.canivoreSignals.addSignals(connectedSignals);
    Robot.canivoreSignals.addSignals(currentSignals);
    // Optimize CAN bus utilization
    // The connected signals don't need to be updated very often, since they should always be
    // connected, and if they aren't, it's a big deal.
    BaseStatusSignal.setUpdateFrequencyForAll(10, connectedSignals);
    // Data rate for the current draws only need to be updated at the robot code's 50 hz
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentSignals);
    // I believe Phoenix Swerve calls optimizeBusUtilization() for us? IDR
  }

  private void updateTelemetry(SwerveDriveState state) {
    try {
      // Loctite™️
      queueLock.lock();
      // Add the latest state to the queue
      // Clone the state so that we get a fresh instance that won't get touched by the odometry
      // thread
      stateQueue.addLast(state.clone());
    } finally {
      // Unlock
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

    // Pull the data out of the circular buffers to put them in arrays for logging
    inputs.poseQueue = new Pose2d[stateQueue.size()];
    inputs.modulePositionsQueue = new SwerveModulePosition[stateQueue.size()][4];
    inputs.rawHeadingQueue = new Rotation2d[stateQueue.size()];
    inputs.timestampQueue = new double[stateQueue.size()];
    for (int i = 0; i < stateQueue.size(); ++i) {
      final var state = stateQueue.get(i);
      inputs.poseQueue[i] = state.Pose;
      inputs.modulePositionsQueue[i] = state.ModulePositions;
      inputs.rawHeadingQueue[i] = state.RawHeading;
      inputs.timestampQueue[i] = state.Timestamp;
    }

    if (stateQueue.size() != 0) {
      // Grab the newsest state
      final var state = stateQueue.get(stateQueue.size() - 1);

      // Fill in the data with the latest state
      inputs.Speeds = state.Speeds;
      inputs.ModuleStates = state.ModuleStates;
      inputs.ModuleTargets = state.ModuleTargets;
      inputs.ModulePositions = state.ModulePositions;

      pose = state.Pose;
    }

    // Clear the buffer
    stateQueue.clear();

    for (int i = 0; i < 4; i++) {
      inputs.driveMotorConnected[i] = connectedSignals[i * 3].getStatus().isOK();
      inputs.steerMotorConnected[i] = connectedSignals[i * 3 + 1].getStatus().isOK();
      inputs.encoderConnected[i] = connectedSignals[i * 3 + 2].getStatus().isOK();
      inputs.driveStatorCurrentAmps[i] = currentSignals[i * 4].getValueAsDouble();
      inputs.driveSupplyCurrentAmps[i] = currentSignals[i * 4 + 1].getValueAsDouble();
      inputs.steerStatorCurrentAmps[i] = currentSignals[i * 4 + 2].getValueAsDouble();
      inputs.steerSupplyCurrentAmps[i] = currentSignals[i * 4 + 3].getValueAsDouble();
    }
    inputs.timeOffsetSeconds = Utils.fpgaToCurrentTime(0);
  }

  // updateOdometry is a noop in real/sim since we can use the real pose estimator directly

  @Override
  public Pose2d getPose() {
    return pose;
  }
}
