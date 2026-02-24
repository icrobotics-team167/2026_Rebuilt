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

public class TurretFeederIOPhoenix implements TurretFeederIO {
  private final TalonFX motor;
  private final int TURRET_FEEDER_ID = 0; // TODO: Find this value
  private final BaseStatusSignal statorSignal, supplySignal;
  private final double TURRET_FEEDER_DEFAULT_VOLTAGE = 12.0;

  public TurretFeederIOPhoenix() {
    motor = new TalonFX(TURRET_FEEDER_ID);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal);
    Robot.canivoreSignals.addSignals(statorSignal, supplySignal);
  }

  @Override
  public void run() {
    motor.setVoltage(TURRET_FEEDER_DEFAULT_VOLTAGE);
  }

  @Override
  public void runReverse() {
    motor.setVoltage(-TURRET_FEEDER_DEFAULT_VOLTAGE);
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
  }

  @Override
  public void updateInputs(TurretFeederIOInputs inputs) {
    inputs.statorCurrentAmps = statorSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplySignal.getValueAsDouble();
  }
}
