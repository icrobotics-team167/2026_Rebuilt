// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.cotc.Robot;

public class FlywheelIOPhoenix implements FlywheelIO {
  private static final int MOTOR_0_ID = 2;
  private static final int MOTOR_1_ID = 3;

  private final TalonFX motor0, motor1;
  private final BaseStatusSignal velocity,
      motor0StatorCurrent,
      motor1StatorCurrent,
      motor0SupplyCurrent,
      motor1SupplyCurrent;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final CoastOut stopRequest = new CoastOut();

  public FlywheelIOPhoenix() {
    motor0 = new TalonFX(MOTOR_0_ID, Robot.rioBus);
    motor1 = new TalonFX(MOTOR_1_ID, Robot.rioBus);

    var config = new TalonFXConfiguration();
    // Coast when idle to preserve angular momentum
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // We need as much torque as we can get, so no stator limit and generous supply limit.
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 80;

    // 1:2 upduction
    config.Feedback.SensorToMechanismRatio = 0.5;
    // The default filtering of a 1ms time constant is very noisy and causes the velocity to
    // oscillate, so we use a 4ms time constant to smooth it out.
    config.Feedback.VelocityFilterTimeConstant = 0.004;
    config.Slot0.kS = 0.363636;
    config.Slot0.kV = 0.06060606;
    config.Slot0.kP = 0.3;
    // We never want the motor to apply torque backwards, and this creates a bang-bang-like
    // behavior by allowing forward voltage but not reverse voltage.
    config.Voltage.PeakReverseVoltage = 0;

    // Left Side
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor0.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor1.getConfigurator().apply(config);

    velocity = motor0.getVelocity(false);

    motor0StatorCurrent = motor0.getStatorCurrent(false);
    motor1StatorCurrent = motor1.getStatorCurrent(false);

    motor0SupplyCurrent = motor0.getSupplyCurrent(false);
    motor1SupplyCurrent = motor1.getSupplyCurrent(false);

    // Optimize CAN bus utilization
    // Data rate for the current draws only need to be updated at the robot code's 50 hz
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocity,
        motor0StatorCurrent,
        motor1StatorCurrent,
        motor0SupplyCurrent,
        motor1SupplyCurrent);

    Robot.rioSignals.addSignals(
        velocity,
        motor0StatorCurrent,
        motor1StatorCurrent,
        motor0SupplyCurrent,
        motor1SupplyCurrent);

    // Everything else can have a slow data rate, but we don't want zero since it can sometimes
    // be useful
    ParentDevice.optimizeBusUtilizationForAll(5, motor0, motor1);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velRotPerSec = velocity.getValueAsDouble();

    inputs.motor0StatorCurrentAmps = motor0StatorCurrent.getValueAsDouble();
    inputs.motor1StatorCurrentAmps = motor1StatorCurrent.getValueAsDouble();

    inputs.motor0SupplyCurrentAmps = motor0SupplyCurrent.getValueAsDouble();
    inputs.motor1SupplyCurrentAmps = motor1SupplyCurrent.getValueAsDouble();
  }

  @Override
  public void runVel(double velRotPerSec) {
    motor0.setControl(velocityRequest.withVelocity(velRotPerSec));
    motor1.setControl(velocityRequest.withVelocity(velRotPerSec));
  }

  @Override
  public void stop() {
    motor0.setControl(stopRequest);
    motor1.setControl(stopRequest);
  }
}
