// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.cotc.Robot;

public class BeltFloorIOPhoenix implements BeltFloorIO {
  private final TalonFX motor;
  private final int BELT_FLOOR_ID = 1;
  private final double BELT_FLOOR_DEFAULT_VOLTAGE = 12.0;
  private final BaseStatusSignal statorSignal, supplySignal, motorVelocitySignal;

  public BeltFloorIOPhoenix() {
    motor = new TalonFX(BELT_FLOOR_ID, Robot.rioBus);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 10;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    motorVelocitySignal = motor.getVelocity(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal, motorVelocitySignal);
    Robot.rioSignals.addSignals(statorSignal, supplySignal, motorVelocitySignal);
  }

  @Override
  public void run() {
    motor.setVoltage(BELT_FLOOR_DEFAULT_VOLTAGE);
  }

  @Override
  public void runReverse() {
    motor.setVoltage(-BELT_FLOOR_DEFAULT_VOLTAGE);
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
  }

  @Override
  public void updateInputs(BeltFloorIOInputs inputs) {
    inputs.statorCurrentAmps = statorSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplySignal.getValueAsDouble();
    inputs.motorVelocity = motorVelocitySignal.getValueAsDouble();
  }
}
