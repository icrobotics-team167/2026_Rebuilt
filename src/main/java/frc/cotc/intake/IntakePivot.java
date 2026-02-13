// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  private final DigitalInput beamBreakSensor;
  private final int BEAM_BREAK_SENSOR_ID = 0; // Placeholder ID for beamBreakSensor

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    beamBreakSensor = new DigitalInput(BEAM_BREAK_SENSOR_ID);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); // 1, Get new data from hardware
    Logger.processInputs("Intake", inputs);
  }

  public Command intake() {
    return run(io::run).finallyDo(io::stop);
  }

  public Command outtake() {
    return run(io::runReverse).finallyDo(io::stop);
  }
}
