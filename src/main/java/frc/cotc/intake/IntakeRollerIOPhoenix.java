// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.cotc.Robot;

public class IntakeRollerIOPhoenix implements IntakeRollerIO {
  private final TalonFX intakeMotor;
  private final int INTAKE_ID = 9;

  private final BaseStatusSignal statorSignal, supplySignal;

  public IntakeRollerIOPhoenix() {
    intakeMotor = new TalonFX(INTAKE_ID, Robot.rioBus);
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    intakeMotor.getConfigurator().apply(config);

    statorSignal = intakeMotor.getStatorCurrent(false);
    supplySignal = intakeMotor.getSupplyCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal);
    Robot.rioSignals.addSignals(statorSignal, supplySignal);
    intakeMotor.optimizeBusUtilization(5);
  }

  @Override
  public void run() {
    intakeMotor.set(-1);
  }

  @Override
  public void runReverse() {
    intakeMotor.set(1);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.statorCurrentAmps = statorSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplySignal.getValueAsDouble();
  }

  @Override
  public void stop() {
    intakeMotor.setVoltage(0);
  }
}
