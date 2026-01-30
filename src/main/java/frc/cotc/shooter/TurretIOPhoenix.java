// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;

public class TurretIOPhoenix implements TurretIO {
  private final CANcoder encoder;
  private final TalonFX motor;
  private final int TURRET_ID = 0; // placeholder
  private final int TURRET_ENCODER_ID = 1; // placeHolder
  private final int TURRET_DEFAULT_VOLTAGE = 12;
  private final BaseStatusSignal posSignal, velSignal, statorSignal, supplySignal;

  public TurretIOPhoenix() {
    encoder = new CANcoder(TURRET_ENCODER_ID);
    motor = new TalonFX(TURRET_ID);
    var turretConfig = new TalonFXConfiguration();
    turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretConfig.CurrentLimits.StatorCurrentLimit = 80;
    turretConfig.CurrentLimits.SupplyCurrentLimit = 60;
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turretConfig.Feedback.FeedbackRemoteSensorID = TURRET_ENCODER_ID;
    motor.getConfigurator().apply(turretConfig);

    var encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);

    posSignal = encoder.getAbsolutePosition(false);
    velSignal = motor.getVelocity(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, posSignal, velSignal, statorSignal, supplySignal);
    Robot.canivoreSignals.addSignals(posSignal, velSignal, statorSignal, supplySignal);
  }

  @Override
  public void run() {
    motor.setVoltage(TURRET_DEFAULT_VOLTAGE);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.statorCurrentAmps = statorSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplySignal.getValueAsDouble();
    inputs.omegaRadPerSec = Units.rotationsToRadians(velSignal.getValueAsDouble());
    inputs.thetaRad = Units.rotationsToRadians(velSignal.getValueAsDouble());
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
  }

  private final PositionVoltage controlSignal = new PositionVoltage(0);

  @Override
  public void runYaw(double thetaRad, double omegaRadPerSec) {
    motor.setControl(
        controlSignal
            .withPosition(Units.radiansToRotations(thetaRad))
            .withPosition(Units.radiansToRotations(omegaRadPerSec)));
  }
}
