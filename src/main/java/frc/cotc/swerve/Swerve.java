// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Robot;
import frc.cotc.vision.AprilTagPoseEstimator;
import frc.cotc.vision.AprilTagPoseEstimatorIOPhoton;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  private final AprilTagPoseEstimator[] aprilTagPoseEstimators;

  public Swerve(SwerveIO io, AprilTagPoseEstimator.IO[] cameraIOs) {
    this.io = io;

    // capture initial state and set up odometry
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    io.updateOdometry(inputs);

    aprilTagPoseEstimators = new AprilTagPoseEstimator[cameraIOs.length];
    for (int i = 0; i < cameraIOs.length; i++) {
      aprilTagPoseEstimators[i] =
          new AprilTagPoseEstimator(
              cameraIOs[i].io(),
              cameraIOs[i].name(),
              (timestamp) -> io.getPose().getRotation(),
              io::getPose);
    }
  }

  @Override
  public void periodic() {
    // Update and process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    // Update odometry using those inputs
    io.updateOdometry(inputs);

    if (Robot.mode == Robot.Mode.SIM) {
      AprilTagPoseEstimatorIOPhoton.Sim.update();
    }
    for (AprilTagPoseEstimator aprilTagPoseEstimator : aprilTagPoseEstimators) {
      var estimates = aprilTagPoseEstimator.poll();

      for (var estimate : estimates) {
        io.addVisionMeasurement(
            estimate.pose(),
            estimate.timestamp(),
            VecBuilder.fill(
                estimate.translationalStdDevs(),
                estimate.translationalStdDevs(),
                estimate.rotationalStdDevs()));
      }
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
    return run(
        () ->
            io.setControl(
                fieldCentricDrive
                    .withVelocityX(vx.getAsDouble() * maxLinearSpeedMetersPerSecond)
                    .withVelocityY(vy.getAsDouble() * maxLinearSpeedMetersPerSecond)
                    .withRotationalRate(omega.getAsDouble() * maxAngularSpeedRadiansPerSecond)));
  }
}
