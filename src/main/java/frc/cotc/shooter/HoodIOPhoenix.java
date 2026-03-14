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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;

public class HoodIOPhoenix implements HoodIO {
  private final TalonFX motor;

  private final int HOOD_MOTOR_ID = 15;
  private final int HOOD_ENCODER_ID = 0;

  private final double ROTOR_TO_SENSOR_RATIO =
      (154.0 / 10.0) * (28.0 / 30.0) * (30.0 / 18.0) * (18.0 / 10.0) / (308.0 / 10.0);
  private final double GEAR_RATIO = 154.0 / 5;

  private final BaseStatusSignal posSignal, statorSignal, supplySignal;

  public HoodIOPhoenix() {
    motor = new TalonFX(HOOD_MOTOR_ID, Robot.rioBus);
    var encoder = new CANcoder(HOOD_ENCODER_ID, Robot.rioBus);

    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = HOOD_ENCODER_ID;
    motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    motorConfig.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    motor.getConfigurator().apply(motorConfig);

    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(encoderConfig);

    posSignal = encoder.getAbsolutePosition(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(50, posSignal, statorSignal, supplySignal);
    Robot.rioSignals.addSignals(posSignal, statorSignal, supplySignal);
    ParentDevice.optimizeBusUtilizationForAll(5, motor, encoder);
  }

  @Override
  public void updateInputs(HoodIOInputs hoodIOInputs) {
    hoodIOInputs.thetaRad = Units.rotationsToRadians(posSignal.getValueAsDouble());
    hoodIOInputs.motorStatorCurrentAmps = statorSignal.getValueAsDouble();
    hoodIOInputs.motorSupplyCurrentAmps = supplySignal.getValueAsDouble();
  }

  private final PositionVoltage controlSignal = new PositionVoltage(0);

  @Override
  public void runPitch(double thetaRad) {
    motor.setControl(controlSignal.withPosition(Units.radiansToRotations(thetaRad)));
  }
}
