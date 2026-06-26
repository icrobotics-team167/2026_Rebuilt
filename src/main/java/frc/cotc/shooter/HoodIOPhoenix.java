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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;

public class HoodIOPhoenix implements HoodIO {
  private final TalonFX motor;

  private final int HOOD_MOTOR_ID = 15;
  private final int HOOD_ENCODER_ID = 0;
  private final double SENSOR_TO_MECHANISM_RATIO = (18.0 / 36.0) * (154.0 / 10.0);

  private final BaseStatusSignal posSignal, statorSignal, supplySignal;

  public HoodIOPhoenix() {
    motor = new TalonFX(HOOD_MOTOR_ID, Robot.rioBus);
    var encoder = new CANcoder(HOOD_ENCODER_ID, Robot.rioBus);

    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = HOOD_ENCODER_ID;
    motorConfig.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
    // Unlike most stuff on the robot, we did care about not breaking the hood, so we limited
    // torque by limiting stator current
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    // These are fairly aggressive PID gains. I honestly wanted to go faster, but the speed at
    // which it moved worried Michael.
    motorConfig.Slot0.kP = 100;
    motorConfig.Slot0.kD = 0.1;
    motorConfig.Slot0.kS = .3;
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    // Use the software limits to prevent the hood from going too far in either direction.
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.048828;
    motor.getConfigurator().apply(motorConfig);

    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = -0.86865234375;
    encoder.getConfigurator().apply(encoderConfig);

    posSignal = motor.getPosition(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);

    // Optimize CAN bus utilization
    // The CANcoder's absolute position signal needs to be sent at a high frequency, since the
    // Kraken uses it to keep its position tracking in line.
    encoder.getAbsolutePosition(false).setUpdateFrequency(250);
    // Data rate for the current position and the current draws only need to be updated at the
    // robot code's 50 hz
    BaseStatusSignal.setUpdateFrequencyForAll(50, posSignal, statorSignal, supplySignal);
    Robot.rioSignals.addSignals(posSignal, statorSignal, supplySignal);
    // Everything else can have a slow data rate, but we don't want zero since it can sometimes
    // be useful
    ParentDevice.optimizeBusUtilizationForAll(5, motor, encoder);
  }

  private final double offset = Units.degreesToRadians(55);

  @Override
  public void updateInputs(HoodIOInputs hoodIOInputs) {
    hoodIOInputs.thetaRad = offset + Units.rotationsToRadians(posSignal.getValueAsDouble());
    hoodIOInputs.motorStatorCurrentAmps = statorSignal.getValueAsDouble();
    hoodIOInputs.motorSupplyCurrentAmps = supplySignal.getValueAsDouble();
  }

  private final PositionVoltage controlSignal = new PositionVoltage(0);

  @Override
  public void runPitch(double thetaRad) {
    motor.setControl(controlSignal.withPosition(Units.radiansToRotations(thetaRad - offset)));
  }
}
