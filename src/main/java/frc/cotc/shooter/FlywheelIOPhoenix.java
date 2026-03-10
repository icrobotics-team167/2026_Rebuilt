// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.cotc.Robot;

public class FlywheelIOPhoenix implements FlywheelIO {
  private static final int MOTOR_0_ID = 2; // TODO: Update Can IDs
  private static final int MOTOR_1_ID = 3;

  private final TalonFX motor0, motor1;
  private final BaseStatusSignal velocity,
      motor0StatorCurrent,
      motor1StatorCurrent,
      motor0SupplyCurrent,
      motor1SupplyCurrent;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut stopRequest = new VoltageOut(0);

  public FlywheelIOPhoenix() {
    motor0 = new TalonFX(MOTOR_0_ID, Robot.rioBus);
    motor1 = new TalonFX(MOTOR_1_ID, Robot.rioBus);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.Slot0.kV = 12.0 / 110.0;
    config.Slot0.kP = .5;

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

    ParentDevice.optimizeBusUtilizationForAll();
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
  }
}
