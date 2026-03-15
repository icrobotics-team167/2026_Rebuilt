// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  private static final double EXTENDED_ANGLE = Units.degreesToRadians(95);
  private static final double RETRACTED_ANGLE = Units.degreesToRadians(10);

  private final PIDController pidController = new PIDController(1.0, 0.0, 0.0);
  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);

  private double targetAngleRad = EXTENDED_ANGLE;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.setDefaultCommand(extend());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
    Logger.recordOutput("IntakePivot/TargetAngleRad", targetAngleRad);
  }

  private Command goToPos(double posRad) {
    return run(() -> {
          targetAngleRad = posRad;
          double pidVolts = pidController.calculate(inputs.pivotAngleRad, targetAngleRad);
          double ffVolts = feedforward.calculate(targetAngleRad, 0);
          io.run(pidVolts + ffVolts);
        })
        .finallyDo(io::stop);
  }

  public Command extend() {
    return goToPos(EXTENDED_ANGLE).withName("Extend");
  }

  public Command retract() {
    return goToPos(RETRACTED_ANGLE).withName("Retract");
  }
}
