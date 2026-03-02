// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.encoder.SplineEncoder;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;

public class IntakePivotIOPhoenix implements IntakePivotIO {
  private final SplineEncoder encoder;
  private final TalonFX intakePivotMotor1;
  private final TalonFX intakePivotMotor2;
  private final int ENCODER_ID = 2; // Placeholder ID for encoder
  private final double ENCODER_OFFSET_ROTATIONS = 0.0;
  private final int INTAKE_ID_1 = 0; // Placeholder ID for intakeMotor1
  private final int INTAKE_ID_2 = 1; // Placeholder ID for intakeMotor2
  private final double INTAKE_DEFAULT_VOLTAGE = 12.0;

  private final BaseStatusSignal statorSignal1, supplySignal1, statorSignal2, supplySignal2;

  public IntakePivotIOPhoenix() {
    encoder = new SplineEncoder(ENCODER_ID);

    intakePivotMotor1 = new TalonFX(INTAKE_ID_1);
    intakePivotMotor2 = new TalonFX(INTAKE_ID_2);
    var config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakePivotMotor1.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakePivotMotor2.getConfigurator().apply(config);

    statorSignal1 = intakePivotMotor2.getStatorCurrent(false);
    statorSignal2 = intakePivotMotor2.getStatorCurrent(false);
    supplySignal1 = intakePivotMotor2.getSupplyCurrent(false);
    supplySignal2 = intakePivotMotor2.getSupplyCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, statorSignal1, statorSignal2, supplySignal1, supplySignal2);
    Robot.canivoreSignals.addSignals(statorSignal1, statorSignal2, supplySignal1, supplySignal2);

    intakePivotMotor1.setPosition(encoder.getRawAngle() - ENCODER_OFFSET_ROTATIONS);
    intakePivotMotor2.setPosition(encoder.getRawAngle() - ENCODER_OFFSET_ROTATIONS);
  }

  private final PositionVoltage controlSignal = new PositionVoltage(0);

  @Override
  public void run(double thetaRad) {
    intakePivotMotor1.setControl(controlSignal.withPosition(Units.radiansToRotations(thetaRad)));
    intakePivotMotor2.setControl(controlSignal.withPosition(Units.radiansToRotations(thetaRad)));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.statorCurrentAmps1 = statorSignal1.getValueAsDouble();
    inputs.statorCurrentAmps2 = statorSignal2.getValueAsDouble();
    inputs.supplyCurrentAmps1 = supplySignal1.getValueAsDouble();
    inputs.supplyCurrentAmps2 = supplySignal2.getValueAsDouble();
    inputs.thetaRad = Units.rotationsToRadians(encoder.getRawAngle() - ENCODER_OFFSET_ROTATIONS);
  }

  @Override
  public void stop() {
    intakePivotMotor1.setVoltage(0);
    intakePivotMotor2.setVoltage(0);
  }
}
