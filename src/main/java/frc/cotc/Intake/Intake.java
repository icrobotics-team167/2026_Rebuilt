// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private IntakeIOInputsAutoLogged inputs;
  private final DigitalInput beamBreakSensor;
  private final int BEAM_BREAK_SENSOR_ID = 0; // Placeholder ID for beamBreakSensor

  @Override
  public void periodic() {
    io.updateInputs(inputs); // 1, Get new data from hardware
    Logger.processInputs("Intake", inputs);
  }

  public Intake(IntakeIO io) {
    this.io = io;
    beamBreakSensor = new DigitalInput(BEAM_BREAK_SENSOR_ID);
  }

  public Command getIntakeCommand() {
    return run(io::run).finallyDo(io::stop);
  }

  public Command getIntakeOutCommand() {
    return run(io::runReverse).finallyDo(io::stop);
  }

  public boolean hasGamePiece() {
    return !beamBreakSensor.get();
  }

  public Command outtakeUntilGamepiece() {
    return getIntakeOutCommand().until(this::hasGamePiece).withName("Outtake Until Gamepiece");
  }
}
