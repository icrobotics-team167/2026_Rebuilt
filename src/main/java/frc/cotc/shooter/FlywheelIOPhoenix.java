// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOPhoenix implements FlywheelIO {
  private static final int MOTOR_0_ID = 0; // TODO: Update Can IDs
  private static final int MOTOR_1_ID = 1;
  private static final int MOTOR_2_ID = 2;
  private static final int MOTOR_3_ID = 3;

  private final TalonFX motor0;
  private final TalonFX motor1;
  private final TalonFX motor2;
  private final TalonFX motor3;

  private final StatusSignal<AngularVelocity> vel0;
  private final StatusSignal<Voltage> volts;
  private final StatusSignal<Current> stat0, stat1, stat2, stat3;
  private final StatusSignal<Current> sup0, sup1, sup2, sup3;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut stopRequest = new VoltageOut(0);

  private final InterpolatingDoubleTreeMap mpsToRpsMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap rpsToMpsMap = new InterpolatingDoubleTreeMap();

  public FlywheelIOPhoenix() {
    motor0 = new TalonFX(MOTOR_0_ID);
    motor1 = new TalonFX(MOTOR_1_ID);
    motor2 = new TalonFX(MOTOR_2_ID);
    motor3 = new TalonFX(MOTOR_3_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 240;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.Slot0.kV = 0.12;
    config.Slot0.kP = 0.10;

    mpsToRpsMap.put(0.0, 0.0); // TODO: Get Real Values
    mpsToRpsMap.put(10.0, 15.0);
    mpsToRpsMap.put(30.0, 45.0);

    rpsToMpsMap.put(0.0, 0.0); // TODO: Get Real Values
    rpsToMpsMap.put(15.0, 10.0);
    rpsToMpsMap.put(45.0, 30.0);

    //  Left Side
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor0.getConfigurator().apply(config);
    motor1.getConfigurator().apply(config);

    //  Right Side
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor2.getConfigurator().apply(config);
    motor3.getConfigurator().apply(config);

    //  All motors follow motor 0
    motor1.setControl(new StrictFollower(MOTOR_0_ID));
    motor2.setControl(new StrictFollower(MOTOR_0_ID));
    motor3.setControl(new StrictFollower(MOTOR_0_ID));

    vel0 = motor0.getVelocity();
    volts = motor0.getMotorVoltage();

    stat0 = motor0.getStatorCurrent();
    stat1 = motor1.getStatorCurrent();
    stat2 = motor2.getStatorCurrent();
    stat3 = motor3.getStatorCurrent();

    sup0 = motor0.getSupplyCurrent();
    sup1 = motor1.getSupplyCurrent();
    sup2 = motor2.getSupplyCurrent();
    sup3 = motor3.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, vel0, stat0, stat1, stat2, stat3, sup0, sup1, sup2, sup3);

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, volts);

    ParentDevice.optimizeBusUtilizationForAll();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(vel0, volts, stat0, stat1, stat2, stat3, sup0, sup1, sup2, sup3);

    double currentRps = vel0.getValueAsDouble();

    inputs.projectileVelMetersPerSec = rpsToMpsMap.get(currentRps);
    inputs.appliedVolts = volts.getValueAsDouble();

    inputs.motor0StatorCurrentAmps = stat0.getValueAsDouble();
    inputs.motor1StatorCurrentAmps = stat1.getValueAsDouble();
    inputs.motor2StatorCurrentAmps = stat2.getValueAsDouble();
    inputs.motor3StatorCurrentAmps = stat3.getValueAsDouble();

    inputs.motor0SupplyCurrentAmps = sup0.getValueAsDouble();
    inputs.motor1SupplyCurrentAmps = sup1.getValueAsDouble();
    inputs.motor2SupplyCurrentAmps = sup2.getValueAsDouble();
    inputs.motor3SupplyCurrentAmps = sup3.getValueAsDouble();
  }

  @Override
  public void runVel(double projectileVelMetersPerSec) {
    double targetRps = mpsToRpsMap.get(projectileVelMetersPerSec);
    motor0.setControl(velocityRequest.withVelocity(targetRps));
  }

  @Override
  public void stop() {
    motor0.setControl(stopRequest);
  }
}
