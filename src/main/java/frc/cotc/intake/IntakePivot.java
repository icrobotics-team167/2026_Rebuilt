// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); // 1, Get new data from hardware
    Logger.processInputs("Intake", inputs);
  }

  public Command extend() {
    return runAngle(0, "Extend");
  }

  public Command retract() {
    return runAngle(Math.PI / 2, "Retract");
  }

  public Command agitate() {
    double intervalSeconds = 0.5; // Placeholder speed

    return Commands.repeatingSequence(
        extend().withTimeout(intervalSeconds),
        retract().withTimeout(intervalSeconds)
    )
    .finallyDo(io::stop)
    .withName("Agitate");
  }

  private Command runAngle(double angleRad, String name) {
    return run(() -> io.run(angleRad)).finallyDo(io::stop);
  }
}
