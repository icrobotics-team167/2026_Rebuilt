// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
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
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
  }

  public Command extend() {
    return run(() -> io.run(12)).until(this::isStalled).finallyDo(() -> io.run(0)).andThen(idle());
  }

  public Command retract() {
    return run(() -> io.run(-12)).until(this::isStalled).andThen(run(() -> io.run(-1)));
  }

  private final Debouncer debouncer = new Debouncer(0.5);

  private boolean isStalled() {
    return debouncer.calculate(
        Math.abs(inputs.statorCurrentAmps) > 40 && Math.abs(inputs.velocityRotPerSec) < 20);
  }

  public Command agitate() {
    double intervalSeconds = 0.5; // Placeholder speed

    return repeatingSequence(
            extend().withTimeout(intervalSeconds), retract().withTimeout(intervalSeconds))
        .withName("Agitate");
  }
}
