// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.climb;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPoseSupplier;

  private final double kClimbTimeoutSeconds = 3.0;

  Climb(ClimbIO io, Supplier<Pose2d> robotPoseSupplier) {
    this.io = io;
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  Command deploy() {
    return runOnce(()-> Logger.recordOutput("Climber/State", "DEPLOYING"))
      .andThen(run(io::deploy))
      .finallyDo(io::stop)
      .withName("Deploy");
  }

  Command retract() { 
    return runOnce(()-> Logger.recordOutput("Climber/State", "RETRACTING"))
      .andThen(run(io::retract))
      .finallyDo(io::stop)
      .withName("Retract");
  }

  Command climb() {
    return runOnce(()-> Logger.recordOutput("Climber/State", "CLIMBING"))
      .andThen(run(io::climb))
      .finallyDo(io::stop)
      .withName("Climb");
  }

  Command doClimb(Pose2d target, double xyTolMeters, double thetaTolRad) {
    return runOnce(() -> Logger.recordOutput("Climber/State", "CLIMBING"))
      .andThen(
        waitUntil(() -> isPoseNear(robotPoseSupplier.get(), target, xyTolMeters, thetaTolRad))
          .andThen(run(io::climb))
          .until(() -> inputs.isAtTop)
          .withTimeout(kClimbTimeoutSeconds)
      )
      .finallyDo(interrupted -> {
        io.stop();
        if (interrupted) {
          Logger.recordOutput("Climber/State", "FAILED");
          Logger.recordOutput("Climber/FailReason", "TIMEOUT");
        } else {
          Logger.recordOutput("Climber/State", "CLIMBED");
        }
      });
  }

  public static boolean isPoseNear(Pose2d pose, Pose2d target, double xyToleranceMeters, double thetaToleranceRad) {
    Translation2d dTrans = pose.getTranslation().minus(target.getTranslation());
    double dTheta = pose.getRotation().minus(target.getRotation()).getRadians();

    return dTrans.getNorm() <= xyToleranceMeters && Math.abs(dTheta) <= thetaToleranceRad;
  }
}
