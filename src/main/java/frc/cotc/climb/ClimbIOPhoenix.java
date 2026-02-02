// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;
import frc.cotc.swerve.TunerConstants;

public class ClimbIOPhoenix implements ClimbIO {

  private final TalonFX motor;
  private final CANcoder encoder;
  private final int CLIMB_MOTOR_ID = 0; // Placeholder CAN ID
  private final int CLIMB_ENCODER_ID = 1; // Placeholder CAN ID
  private final double DEPLOY_VOLTAGE = 0.0; // Placeholder voltage
  private final double RETRACT_VOLTAGE = 0.0; // Placeholder voltage
  private final double CLIMB_VOLTAGE = 0.0; // Placeholder voltage

  private final BaseStatusSignal positionSignal;

  public ClimbIOPhoenix() {
    encoder = new CANcoder(CLIMB_ENCODER_ID, TunerConstants.kCANBus);
    motor = new TalonFX(CLIMB_MOTOR_ID, TunerConstants.kCANBus);
    encoder.optimizeBusUtilization();

    var encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);

    var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = 80; // Placeholder
    motorConfig.CurrentLimits.StatorCurrentLimit = 60; // Placeholder
    motor.getConfigurator().apply(motorConfig);

    positionSignal = encoder.getAbsolutePosition(false);
    positionSignal.setUpdateFrequency(50);

    Robot.canivoreSignals.addSignals(positionSignal);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.posRad = Units.rotationsToRadians(positionSignal.getValueAsDouble());
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void deploy() {
    motor.setVoltage(DEPLOY_VOLTAGE);
  }

  @Override
  public void retract() {
    motor.setVoltage(RETRACT_VOLTAGE);
  }

  @Override
  public void climb() {
    motor.setVoltage(CLIMB_VOLTAGE);
  }
}
