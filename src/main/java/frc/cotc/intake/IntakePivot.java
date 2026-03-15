// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  // TODO: Setpoints
  private static final double EXTENDED_ANGLE = Units.degreesToRadians(95);
  private static final double RETRACTED_ANGLE = Units.degreesToRadians(10);
  // TODO: Tune these
  private final PIDController pidController = new PIDController(1.0, 0.0, 0.0);
  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private double targetAngleRad = EXTENDED_ANGLE;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    pidController.setTolerance(Units.degreesToRadians(2.0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
    double pidVolts = pidController.calculate(inputs.pivotAngleRad, targetAngleRad);
    double ffVolts = feedforward.calculate(targetAngleRad, 0);
    io.run(pidVolts + ffVolts);
    Logger.recordOutput("IntakePivot/TargetAngleRad", targetAngleRad);
    Logger.recordOutput("IntakePivot/AtSetpoint", pidController.atSetpoint());
  }

  public Command retractWhileHeld() {
    return run(() -> targetAngleRad = RETRACTED_ANGLE)
        .finallyDo(() -> targetAngleRad = EXTENDED_ANGLE)
        .withName("RetractWhileHeld");
  }

  public Command extend() {
    return runOnce(() -> targetAngleRad = EXTENDED_ANGLE)
        .andThen(waitUntil(pidController::atSetpoint))
        .withName("Extend");
  }

  public Command retract() {
    return runOnce(() -> targetAngleRad = RETRACTED_ANGLE)
        .andThen(waitUntil(pidController::atSetpoint))
        .withName("Retract");
  }

  public Command agitate() {
    double intervalSeconds = 0.5;
    return repeatingSequence(
            extend().withTimeout(intervalSeconds), retract().withTimeout(intervalSeconds))
        .withName("Agitate");
  }
}
