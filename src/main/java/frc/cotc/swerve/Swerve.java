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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  public Swerve(SwerveIO io) {
    this.io = io;

    // capture initial state and set up odometry
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    io.updateOdometry(inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    io.updateOdometry(inputs);
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

  public Command fakeVision() {
    return Commands.run(
        () ->
            io.addVisionMeasurement(
                new Pose2d(3, 3, Rotation2d.kZero), fpgaToCurrentTime(Timer.getTimestamp())));
  }

  /**
   * Converts an FPGA timestamp to the timebase reported by {@link Utils#getCurrentTimeSeconds()}.
   */
  private double fpgaToCurrentTime(double fpgaTimeSeconds) {
    return fpgaTimeSeconds + inputs.fpgaToCurrentTime;
  }

  /**
   * Converts a timestamp in the timebase reported by {@link Utils#getCurrentTimeSeconds()} to an
   * FPGA timestamp.
   */
  private double currentTimeToFPGATime(double currentTimeSeconds) {
    return currentTimeSeconds - inputs.fpgaToCurrentTime;
  }
}
