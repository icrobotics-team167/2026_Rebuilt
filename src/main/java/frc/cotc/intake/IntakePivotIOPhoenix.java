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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.encoder.SplineEncoder;

import frc.cotc.Robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class IntakePivotIOPhoenix implements IntakePivotIO {
  private final TalonFX motor;
  private final int MOTOR_ID = 8;
  private final SplineEncoder pivotEncoder = new SplineEncoder(0);
  private final BaseStatusSignal statorSignal, supplySignal, velocitySignal;

  public IntakePivotIOPhoenix() {
    motor = new TalonFX(MOTOR_ID, Robot.rioBus);
    var config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 

    motor.getConfigurator().apply(config);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    velocitySignal = motor.getVelocity(false);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        statorSignal,
        supplySignal,
        velocitySignal
    );

    Robot.rioSignals.addSignals(statorSignal, supplySignal, velocitySignal);
  }

  @Override
  public void run(double volts) {
    motor.setVoltage(volts);
  }
  private final double offsetRot = 0;

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.statorCurrentAmps = statorSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplySignal.getValueAsDouble();
    inputs.velocityRotPerSec = velocitySignal.getValueAsDouble();
    inputs.pivotAngleRad = MathUtil.angleModulus(Units.rotationsToRadians(pivotEncoder.getAngle() - offsetRot));
  }
}