// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.encoder.SplineEncoder;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;

public class IntakePivotIOPhoenix implements IntakePivotIO {
  private final SplineEncoder encoder;
  private final TalonFX motor;
  private final int ENCODER_ID = 2; // Placeholder ID for encoder
  private final double ENCODER_OFFSET_ROTATIONS = 0.0;
  private final int MOTOR_ID = 8;

  private final BaseStatusSignal statorSignal, supplySignal;

  public IntakePivotIOPhoenix() {
    encoder = new SplineEncoder(ENCODER_ID);

    motor = new TalonFX(MOTOR_ID, CANBus.roboRIO());
    var config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, statorSignal, supplySignal);
    Robot.rioSignals.addSignals(statorSignal, supplySignal);

    motor.setPosition(encoder.getRawAngle() - ENCODER_OFFSET_ROTATIONS);
  }

  private final PositionVoltage controlSignal = new PositionVoltage(0);

  @Override
  public void run(double thetaRad) {
    motor.setControl(controlSignal.withPosition(Units.radiansToRotations(thetaRad)));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.statorCurrentAmps = statorSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplySignal.getValueAsDouble();
    inputs.thetaRad = Units.rotationsToRadians(encoder.getRawAngle() - ENCODER_OFFSET_ROTATIONS);
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
  }
}
