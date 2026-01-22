// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeIOPhoenix implements IntakeIO {
  private final TalonFX intakeMotor1;
  private final TalonFX intakeMotor2;
  private final int INTAKE_ID_1 = 0; // Placeholder ID for intakeMotor1
  private final int INTAKE_ID_2 = 1; // Placeholder ID for intakeMotor2
  private final double INTAKE_DEFAULT_VOLTAGE = 12.0;
  private final double OUTAKE_DEFAULT_VOLTAGE = -12.0;

  public IntakeIOPhoenix() {

    intakeMotor1 = new TalonFX(INTAKE_ID_1);
    intakeMotor2 = new TalonFX(INTAKE_ID_2);
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    intakeMotor1.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeMotor2.getConfigurator().apply(config);
  }

  @Override
  public void run() {
    intakeMotor1.setVoltage(INTAKE_DEFAULT_VOLTAGE);
    intakeMotor2.setVoltage(INTAKE_DEFAULT_VOLTAGE);
  }

  @Override
  public void runReverse() {
    intakeMotor1.setVoltage(OUTAKE_DEFAULT_VOLTAGE);
    intakeMotor2.setVoltage(OUTAKE_DEFAULT_VOLTAGE);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.statorCurrentAmps1 = intakeMotor1.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps1 = intakeMotor1.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps2 = intakeMotor2.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps2 = intakeMotor2.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void stop() {
    intakeMotor1.setVoltage(0);
    intakeMotor2.setVoltage(0);
  }
}
