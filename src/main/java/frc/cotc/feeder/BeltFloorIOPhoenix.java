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
  private final int BELTFLOOR_ID = 0; // placeholder
  private final double BELTFLOOR_DEFAULT_VOLTAGE = 12.0;
  private final BaseStatusSignal statorSignal, supplySignal, motorSpeedSignal;

  public BeltFloorIOPhoenix() {
    motor = new TalonFX(BELTFLOOR_ID);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    motorSpeedSignal = motor.getAcceleration(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal, motorSpeedSignal);
    Robot.canivoreSignals.addSignals(statorSignal, supplySignal, motorSpeedSignal);
  }

  @Override
  public void run() {
    motor.setVoltage(BELTFLOOR_DEFAULT_VOLTAGE);
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
  }

  @Override
  public void updateInputs(BeltFloorIOInputs inputs) {
    inputs.statorCurrentAmps = statorSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplySignal.getValueAsDouble();
    inputs.motorSpeed = Math.abs(motorSpeedSignal.getValueAsDouble());
  }
}
