// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakePivot pivot;
  private final IntakeRoller roller;

  public Intake(IntakePivotIO pivot, IntakeRollerIO roller) {
    this.pivot = new IntakePivot(pivot);
    this.roller = new IntakeRoller(roller);
  }

  public Command extend() {
    return expose(parallel(pivot.extend(), roller.intake().withTimeout(0.25)), "Extend");
  }

  public Command fastExtend() {
    return expose(parallel(pivot.fastExtend(), roller.intake()), "Fast Extend");
  }

  public Command retract() {
    return expose(parallel(pivot.retract(), roller.intake().withTimeout(0.25)), "Retract");
  }

  public Command intake() {
    return expose(parallel(pivot.extend(), roller.intake()), "Intake");
  }

  public Command outtake() {
    return expose(parallel(pivot.lowAgitate(), roller.outtake()), "Outtake");
  }

  public Command agitate() {
    return expose(parallel(pivot.agitate(), roller.intake()), "Agitate");
  }

  /**
   * Wraps a command such that from the outside, it looks like only the parent subsystem is
   * requiring the command.
   *
   * <p><b>All PUBLICLY AVAILABLE COMMANDS MUST BE RUN THROUGH THIS METHOD OR * ELSE IT'S ENTIRELY
   * UNDEFINED BEHAVIOR.
   *
   * @param internal The command to wrap
   * @return The wrapped command.
   */
  protected Command expose(Command internal, String name) {
    var proxied = internal.withName(name).asProxy();
    proxied.addRequirements(this);
    return proxied;
  }
}
