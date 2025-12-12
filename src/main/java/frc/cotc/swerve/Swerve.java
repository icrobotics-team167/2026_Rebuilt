// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIO.SwerveIOInputs inputs = new SwerveIO.SwerveIOInputs();

  public Swerve(SwerveIO io) {
    this.io = io;

    io.updateInputs(inputs);
    // TODO: Log
  }

  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault()
          .getTable("Swerve")
          .getStructTopic("Estimated Pose", Pose2d.struct)
          .publish();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // TODO: Log
    posePublisher.set(inputs.poseQueue[inputs.poseQueue.length - 1]);
  }

  private final double maxLinearSpeedMetersPerSecond =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularSpeedRadiansPerSecond =
      maxLinearSpeedMetersPerSecond
          / Math.min(
              Math.min(
                  Math.hypot(
                      TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                  Math.hypot(
                      TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
              Math.min(
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
