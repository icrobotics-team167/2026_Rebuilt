// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;

public class TrajectoryCalc {
  private static final double ballDiameter = Units.inchesToMeters(5.91);
  private static final double ballMass = Units.lbsToKilograms(0.5);

  private static Vector<N6> f(Vector<N6> x, Vector<N3> omega) {
    // x' = x'
    // y' = y'
    // z' = z'
    // x" = −F_D(v)/m v̂_x
    // y" = −F_D(v)/m v̂_y
    // z" = −g − F_D(v)/m v̂_z
    //
    // Per https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation:
    //   F_D(v) = ½ρv²C_D A
    //   ρ is the fluid density in kg/m³
    //   v is the velocity magnitude in m/s
    //   C_D is the drag coefficient (dimensionless)
    //   A is the cross-sectional area of a circle in m²
    //   m is the mass in kg
    //   v̂ is the velocity direction unit vector
    var rho = 1.204;
    var v = VecBuilder.fill(x.get(3), x.get(4), x.get(5));
    var v2 = v.get(0) * v.get(0) + v.get(1) * v.get(1) + v.get(2) * v.get(2);
    var C_D = 0.4;
    var r = ballDiameter / 2;
    var A = Math.PI * r * r;
    var m = ballMass;
    var F_D = 0.5 * rho * v2 * C_D * A;

    var v_mag = Math.sqrt(v2);

    var C_L = 0.00025;

    var v_hat = v.div(v_mag);
    var F_M = Vector.cross(omega, v).times(0.5 * rho * C_L * A * v_mag);
    var a = v_hat.times(-F_D / m).plus(F_M).plus(VecBuilder.fill(0, 0, -9.81));
    return VecBuilder.fill(v.get(0), v.get(1), v.get(2), a.get(0), a.get(1), a.get(2));
  }

  private static Vector<N6> rk4(Vector<N6> x, Vector<N3> omega, double dt) {
    var k1 = f(x, omega);
    var k2 = f(x.plus(k1.times(dt / 2)), omega);
    var k3 = f(x.plus(k2.times(dt / 2)), omega);
    var k4 = f(x.plus(k3.times(dt)), omega);
    var dx = (k1.plus(k2.times(2)).plus(k3.times(2)).plus(k4)).times(dt / 6);
    return x.plus(dx);
  }

  public static Pose3d[] simulateShot(Translation3d initialPose, Translation3d initialVelocity) {
    var omegaMag = initialVelocity.getNorm() / ballDiameter;
    var rotatedYaw = initialVelocity.toTranslation2d().getAngle().plus(Rotation2d.kCW_Pi_2);
    var omega = VecBuilder.fill(omegaMag * rotatedYaw.getCos(), omegaMag * rotatedYaw.getSin(), 0);
    var x =
        VecBuilder.fill(
            initialPose.getX(),
            initialPose.getY(),
            initialPose.getZ(),
            initialVelocity.getX(),
            initialVelocity.getY(),
            initialVelocity.getZ());

    var poses = new ArrayList<Pose3d>();
    var t = 0.0;
    var dt = 0.02;
    while (t < 2) {
      poses.add(new Pose3d(x.get(0), x.get(1), x.get(2), Rotation3d.kZero));
      x = rk4(x, omega, dt);
      if (x.get(2) < 0) {
        break;
      }
      t += dt;
    }
    poses.add(new Pose3d(x.get(0), x.get(1), x.get(2), Rotation3d.kZero));
    return poses.toArray(new Pose3d[0]);
  }
}
