// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.cotc.Robot;

public class FlywheelIOPhoenix implements FlywheelIO {
  private static final int MOTOR_0_ID = 0; // TODO: Update Can IDs
  private static final int MOTOR_1_ID = 1;

  private final TalonFX motor0, motor1;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> motor0StatorCurrent, motor1StatorCurrent;
  private final StatusSignal<Current> motor0SupplyCurrent, motor1SupplyCurrent;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut stopRequest = new VoltageOut(0);

  public FlywheelIOPhoenix() {
    motor0 = new TalonFX(MOTOR_0_ID);
    motor1 = new TalonFX(MOTOR_1_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 240;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.Slot0.kV = 0.12;
    config.Slot0.kP = 0.10;

    //  Left Side
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor0.getConfigurator().apply(config);
    motor1.getConfigurator().apply(config);

    //  All motors follow motor 0
    motor1.setControl(new StrictFollower(MOTOR_0_ID));

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

    Robot.canivoreSignals.addSignals(
        velocity,
        motor0StatorCurrent,
        motor1StatorCurrent,
        motor0SupplyCurrent,
        motor1SupplyCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, motor0.getMotorVoltage(false));

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
  }

  @Override
  public void stop() {
    motor0.setControl(stopRequest);
  }
}
