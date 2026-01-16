// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import static org.wpilib.math.autodiff.Variable.hypot;
import static org.wpilib.math.optimization.Constraints.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;
import frc.cotc.vision.AprilTagPoseEstimator;
import java.util.Optional;
import java.util.function.Function;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.autodiff.Slice;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.ExitStatus;

public class ShotSolver {
  private final SimpleMatrix blueTargetWrtField =
      new SimpleMatrix(
          new double[][] {
            new double[] {Units.inchesToMeters(158.6 + (47.0 / 2))},
            new double[] {AprilTagPoseEstimator.tagLayout.getFieldWidth() / 2},
            new double[] {Units.inchesToMeters(72)}
          });
  private final SimpleMatrix redTargetWrtField =
      new SimpleMatrix(
          new double[][] {
            new double[] {
              AprilTagPoseEstimator.tagLayout.getFieldLength()
                  - Units.inchesToMeters(158.6 + (47.0 / 2))
            },
            new double[] {AprilTagPoseEstimator.tagLayout.getFieldWidth() / 2},
            new double[] {Units.inchesToMeters(72)}
          });

  private VariableMatrix lastX;

  public Optional<Pair<Double, Rotation2d>> solve(
      double x, double y, double vx, double vy, double shooterVel) {
    var shooterWrtField =
        new SimpleMatrix(
            new double[][] {
              new double[] {x},
              new double[] {y},
              new double[] {Units.inchesToMeters(20)},
              new double[] {vx},
              new double[] {vy},
              new double[] {0},
            });

    var targetWrtField = Robot.isOnRed() ? redTargetWrtField : blueTargetWrtField;

    var problem = new Problem();

    int N = 20;

    var T = problem.decisionVariable();
    problem.subjectTo(gt(T, 0));
    T.setValue(1);
    var dt = T.div(N);

    var X = problem.decisionVariable(6, N);

    var p = X.get(new Slice(0, 3), Slice.__);
    var p_x = X.get(0, Slice.__);
    var p_y = X.get(1, Slice.__);
    var p_z = X.get(2, Slice.__);

    var v_x = X.get(3, Slice.__);
    var v_y = X.get(4, Slice.__);
    var v_z = X.get(5, Slice.__);

    var x_shooterToTarget = targetWrtField.get(0) - shooterWrtField.get(0);
    var y_shooterToTarget = targetWrtField.get(1) - shooterWrtField.get(1);
    var z_shooterToTarget = targetWrtField.get(2) - shooterWrtField.get(2);

    var magnitude =
        Math.sqrt(
            x_shooterToTarget * x_shooterToTarget
                + y_shooterToTarget * y_shooterToTarget
                + z_shooterToTarget * z_shooterToTarget);

    if (lastX == null) {
      // Initial guess is straight line to target if no previous solve
      for (int k = 0; k <= N; k++) {
        p_x.get(k)
            .setValue(
                MathUtil.interpolate(
                    shooterWrtField.get(0), targetWrtField.get(0), (double) k / (double) N));
        p_y.get(k)
            .setValue(
                MathUtil.interpolate(
                    shooterWrtField.get(1), targetWrtField.get(1), (double) k / (double) N));
        p_z.get(k)
            .setValue(
                MathUtil.interpolate(
                    shooterWrtField.get(2), targetWrtField.get(2), (double) k / (double) N));
        v_x.get(k).setValue(shooterWrtField.get(3) + shooterVel * x_shooterToTarget / magnitude);
        v_y.get(k).setValue(shooterWrtField.get(4) + shooterVel * y_shooterToTarget / magnitude);
        v_z.get(k).setValue(shooterWrtField.get(5) + shooterVel * z_shooterToTarget / magnitude);
      }

    } else {
      // Last solve is initial guess
      var last_p_x = lastX.get(0, Slice.__);
      var last_p_y = lastX.get(1, Slice.__);
      var last_p_z = lastX.get(2, Slice.__);
      var last_v_x = lastX.get(3, Slice.__);
      var last_v_y = lastX.get(4, Slice.__);
      var last_v_z = lastX.get(5, Slice.__);
      for (int k = 0; k <= N; k++) {
        p_x.get(k).setValue(last_p_x.get(k).value());
        p_y.get(k).setValue(last_p_y.get(k).value());
        p_z.get(k).setValue(last_p_z.get(k).value());
        v_x.get(k).setValue(last_v_x.get(k).value());
        v_y.get(k).setValue(last_v_y.get(k).value());
        v_z.get(k).setValue(last_v_z.get(k).value());
      }
    }

    var v0_wrt_shooter = X.get(new Slice(3, 6), 0).minus(shooterWrtField);

    problem.subjectTo(eq(p.get(Slice.__, 0), shooterWrtField));

    for (var k = 0; k < N - 1; k++) {
      var x_k = X.get(Slice.__, k);
      var x_k1 = X.get(Slice.__, k + 1);

      var k1 = f(new VariableMatrix(x_k));
      var k2 = f(x_k.plus(k1.times(dt.div(2))));
      var k3 = f(x_k.plus(k2.times(dt.div(2))));
      var k4 = f(x_k.plus(k3.times(dt)));
      problem.subjectTo(
          eq(x_k1, x_k.plus((k1.plus(k2.times(2)).plus(k3.times(2)).plus(k4)).times(dt.div(6)))));
    }

    problem.subjectTo(eq(p.get(Slice.__, -1), targetWrtField));

    problem.subjectTo(le(hypot(v_x.get(-1), v_y.get(-1)), 6.5));
    problem.subjectTo(lt(v_z.get(-1), 0));

    problem.subjectTo(
        eq(
            (v0_wrt_shooter.get(0).times(v0_wrt_shooter.get(0)))
                .plus(v0_wrt_shooter.get(0).times(v0_wrt_shooter.get(0)))
                .plus(v0_wrt_shooter.get(0).times(v0_wrt_shooter.get(0))),
            shooterVel * shooterVel));

    var status = problem.solve();
    Logger.recordOutput("ShotSolver/Status", status);
    if (status == ExitStatus.SUCCESS) {
      lastX = X;
      var pitch =
          Math.atan2(
              v0_wrt_shooter.get(1).value(),
              Math.hypot(v0_wrt_shooter.get(0).value(), v0_wrt_shooter.get(1).value()));
      var yaw = new Rotation2d(v0_wrt_shooter.get(0).value(), v0_wrt_shooter.get(1).value());
      Logger.recordOutput("ShotSolver/Pitch", pitch);
      Logger.recordOutput("ShotSolver/Yaw", yaw);

      var trajectory = new Pose3d[N];
      for (int i = 0; i < N; i++) {
        trajectory[i] =
            new Pose3d(
                p.get(0, N).value(), p.get(1, N).value(), p.get(2, N).value(), Rotation3d.kZero);
      }

      Logger.recordOutput("ShotSolver/Trajectory", trajectory);
      return Optional.of(new Pair<>(pitch, yaw));
    } else {
      return Optional.empty();
    }
  }

  /** Apply the drag equation to a velocity. */
  private VariableMatrix f(VariableMatrix x) {
    // x' = x'
    // y' = y'
    // z' = z'
    // x" = −a_D(v_x)
    // y" = −a_D(v_y)
    // z" = −g − a_D(v_z)
    //
    // where a_D(v) = ½ρv² C_D A / m
    // (see https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation)
    var rho = 1.204;
    var C_D = 0.5;
    var m = 0.5 / 2.205;
    var r = 5.91 * 0.0254 / 2;
    var A = Math.PI * r * r;
    Function<Variable, Variable> a_D = (v) -> v.times(v).times(0.5 * rho * C_D * A / m);

    var v_x = x.get(3, 0);
    var v_y = x.get(4, 0);
    var v_z = x.get(5, 0);

    return new VariableMatrix(
        new Variable[][] {
          new Variable[] {v_x},
          new Variable[] {v_y},
          new Variable[] {v_z},
          new Variable[] {a_D.apply(v_x).unaryMinus()},
          new Variable[] {a_D.apply(v_y).unaryMinus()},
          new Variable[] {a_D.apply(v_z).unaryMinus().minus(9.81)}
        });
  }

  private void init() {
    solve(0, 4, 0, 0, 10);
  }
}
