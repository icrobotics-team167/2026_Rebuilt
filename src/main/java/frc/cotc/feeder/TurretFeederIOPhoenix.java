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
  private final int TURRET_FEEDER_ID = 7;
  private final BaseStatusSignal statorSignal, supplySignal;

  public TurretFeederIOPhoenix() {
    motor = new TalonFX(TURRET_FEEDER_ID, Robot.rioBus);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal);
    Robot.rioSignals.addSignals(statorSignal, supplySignal);
    motor.optimizeBusUtilization(5);
  }

  @Override
  public void run() {
    motor.set(1);
  }

  @Override
  public void stop() {
    motor.set(0);
  }

  @Override
  public void updateInputs(TurretFeederIOInputs inputs) {
    inputs.statorCurrentAmps = statorSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplySignal.getValueAsDouble();
  }
}
