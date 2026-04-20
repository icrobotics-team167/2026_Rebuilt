// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  private static final double EXTENDED_ANGLE = 0;
  private static final double AGITATE_ANGLE = 1.5;
  private static final double LOW_AGITAGE_ANGLE = 0.4;
  private static final double RETRACTED_ANGLE = 2.274;

  private final PIDController pidController = new PIDController(8, 0.0, 0.1);
  private final PIDController slowPidController = new PIDController(3, 0, 0.05);
  private final ArmFeedforward feedforward = new ArmFeedforward(0.158, 0.242, 0.0);

  private double targetAngleRad = EXTENDED_ANGLE;

  IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
    Logger.recordOutput("IntakePivot/TargetAngleRad", targetAngleRad);
  }

  private Command goToPos(double posRad, boolean slow) {
    return run(() -> {
          targetAngleRad = posRad;
          var controller = slow ? slowPidController : pidController;
          double pidVolts = controller.calculate(inputs.pivotAngleRad, targetAngleRad);
          double ffVolts = feedforward.calculate(targetAngleRad - 0.61, 0);
          Logger.recordOutput("IntakePivot/PIDVolts", pidVolts);
          io.run(pidVolts + ffVolts);
        })
        .finallyDo(io::stop);
  }

  Command extend() {
    return goToPos(EXTENDED_ANGLE, true).withName("Extend");
  }

  Command retract() {
    return goToPos(RETRACTED_ANGLE, false).withName("Retract");
  }

  Command agitate() {
    return repeatingSequence(
            goToPos(AGITATE_ANGLE, false).withTimeout(0.5), extend().withTimeout(0.75))
        .withName("Agitate");
  }

  Command lowAgitate() {
    return repeatingSequence(
            goToPos(LOW_AGITAGE_ANGLE, false).withTimeout(0.4), extend().withTimeout(0.4))
        .withName("Low Agitate");
  }
}
