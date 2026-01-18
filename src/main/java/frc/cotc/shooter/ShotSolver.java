// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import static org.wpilib.math.optimization.Constraints.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.cotc.vision.AprilTagPoseEstimator;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.autodiff.Slice;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.ExitStatus;
import org.wpilib.math.optimization.solver.Options;

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

  private final double shooterHeight;

  public ShotSolver(double shooterHeight) {
    this.shooterHeight = shooterHeight;
  }

  private VariableMatrix f(VariableMatrix x) {
    var v_x = x.get(3, 0);
    var v_y = x.get(4, 0);
    var v_z = x.get(5, 0);
    return new VariableMatrix(
        new Variable[][] {
          {v_x},
          {v_y},
          {v_z},
          {a_D(v_x).unaryMinus()},
          {a_D(v_y).unaryMinus()},
          {a_D(v_z).unaryMinus().minus(9.81)}
        });
  }

  private Variable a_D(Variable v) {
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
    var r = 5.91 * .0254 / 2;
    var A = Math.PI * r * r;

    return v.times(v).times(0.5 * rho * C_D * A / m);
  }

  private final int N = 20;

  public Optional<Pair<Double, Rotation2d>> solve(
      double x, double y, double vx, double vy, double shotVel) {
    var startTime = Timer.getFPGATimestamp();
    var shooterPosWrtField = new SimpleMatrix(new double[][] {{x}, {y}, {shooterHeight}});
    var shooterVelWrtField = new SimpleMatrix(new double[][] {{vx}, {vy}, {0}});
    var targetPosWrtField = blueTargetWrtField;

    // Setup problem
    var problem = new Problem();

    // Set up time and delta time
    var T = problem.decisionVariable();
    problem.subjectTo(ge(T, 0));
    T.setValue(1);
    var dt = T.div(N);

    // Ball state in field frame
    //
    //     [x position]
    //     [y position]
    //     [z position]
    // x = [x velocity]
    //     [y velocity]
    //     [z velocity]
    var X = problem.decisionVariable(6, N);

    var p = X.get(new Slice(0, 3), Slice.__);
    var p_x = X.get(0, Slice.__);
    var p_y = X.get(1, Slice.__);
    var p_z = X.get(2, Slice.__);

    var v = X.get(new Slice(3, 6), Slice.__);
    var v_x = X.get(3, Slice.__);
    var v_y = X.get(4, Slice.__);
    var v_z = X.get(5, Slice.__);

    var v0WrtShooter = v.get(Slice.__, 0).minus(shooterVelWrtField);

    if (lastX == null) {
      Logger.recordOutput("ShotSolver/Seeded", false);
      var uvecShooterToTarget = targetPosWrtField.minus(shooterPosWrtField);
      uvecShooterToTarget.scale(1.0 / uvecShooterToTarget.normF());

      for (int k = 0; k < N; k++) {
        p_x.get(k)
            .setValue(
                MathUtil.interpolate(
                    shooterPosWrtField.get(0, 0), targetPosWrtField.get(0, 0), k / ((double) N)));
        p_y.get(k)
            .setValue(
                MathUtil.interpolate(
                    shooterPosWrtField.get(1, 0), targetPosWrtField.get(1, 0), k / ((double) N)));
        p_z.get(k)
            .setValue(
                MathUtil.interpolate(
                    shooterPosWrtField.get(2, 0), targetPosWrtField.get(2, 0), k / ((double) N)));
        v.get(Slice.__, k).setValue(shooterVelWrtField.plus(uvecShooterToTarget.scale(shotVel)));
      }
    } else {
      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < N; j++) {
          X.get(i, j).setValue(lastX.get(i, j).value());
        }
      }
      Logger.recordOutput("ShotSolver/Seeded", true);
    }

    // Initial pos of ball at the shooter's pos
    problem.subjectTo(eq(p.get(Slice.__, 0), shooterPosWrtField));

    // Set initial velocity of shot
    //
    //   √(v_x² + v_y² + v_z²) = v
    //   v_x² + v_y² + v_z² = v²
    problem.subjectTo(
        eq(
            (v0WrtShooter.get(0).times(v0WrtShooter.get(0)))
                .plus(v0WrtShooter.get(1).times(v0WrtShooter.get(1)))
                .plus(v0WrtShooter.get(2).times(v0WrtShooter.get(2))),
            shotVel * shotVel));

    // RK4 integration to enforce dynamics
    for (int k = 0; k < N - 1; k++) {
      var x_k = new VariableMatrix(X.get(Slice.__, k));
      var x_k1 = X.get(Slice.__, k + 1);

      var k1 = f(x_k);
      var k2 = f(x_k.plus(k1.times(dt.div(2))));
      var k3 = f(x_k.plus(k2.times(dt.div(2))));
      var k4 = f(x_k.plus(k3.times(dt)));
      problem.subjectTo(
          eq(x_k1, x_k.plus((k1.plus(k2.times(2)).plus(k3.times(2)).plus(k4)).times(dt.div(6)))));
    }

    // Final pos of ball at target
    problem.subjectTo(eq(p.get(Slice.__, -1), targetPosWrtField));

    problem.subjectTo(lt(v_z.get(-1), 0));
    //    problem.subjectTo(le(hypot(v_x.get(-1), v_y.get(-1)), v_z.get(-1).times(-1.4)));

    var status =
        problem.solve(
            new Options().withDiagnostics(false).withTolerance(.01).withMaxIterations(1000));
    Logger.recordOutput("ShotCalculator/Status", status.name());
    var trajectory = new Pose3d[N];
    for (int i = 0; i < N; i++) {
      trajectory[i] =
          new Pose3d(p_x.get(i).value(), p_y.get(i).value(), p_z.get(i).value(), Rotation3d.kZero);
    }
    Logger.recordOutput("ShotCalculator/Trajectory", trajectory);
    Logger.recordOutput(
        "ShotCalculator/Trajectory angle at target deg",
        Units.radiansToDegrees(
            Math.atan2(v_z.get(-1).value(), Math.hypot(v_x.get(-1).value(), v_y.get(-1).value()))));
    Logger.recordOutput("ShotCalculator/Time", Timer.getFPGATimestamp() - startTime);
    problem.close();
    if (status == ExitStatus.SUCCESS) {
      lastX = X;
      return Optional.of(
          new Pair<>(
              Math.atan2(
                  v0WrtShooter.get(2).value(),
                  Math.hypot(v0WrtShooter.get(0).value(), v0WrtShooter.get(1).value())),
              new Rotation2d(v0WrtShooter.get(0).value(), v0WrtShooter.get(1).value())));
    }
    return Optional.empty();
  }

  public void init() {
    solve(0, AprilTagPoseEstimator.tagLayout.getFieldWidth() / 2, 0, 0, 8.5);
  }
}
