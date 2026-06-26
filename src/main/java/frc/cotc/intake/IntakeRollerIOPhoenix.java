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
    // We need as much torque as we can get, so no stator limit and generous supply limit.
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    intakeMotor.getConfigurator().apply(config);

    statorSignal = intakeMotor.getStatorCurrent(false);
    supplySignal = intakeMotor.getSupplyCurrent(false);
    // Optimize CAN bus utilization
    // Data rate for the current draws only need to be updated at the robot code's 50 hz
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal);
    Robot.rioSignals.addSignals(statorSignal, supplySignal);
    // Everything else can have a slow data rate, but we don't want zero since it can sometimes
    // be useful
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
