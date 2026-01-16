// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private final HoodIO hoodIO;

  public Shooter(HoodIO hoodIO) {
    this.hoodIO = hoodIO;
  }

  private void runShotAngle() {
    // Algorithm by Eeswhar Kirshnan
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    var shooterToTargetVec = getShooterToTargetVec();
    var distanceMeters = shooterToTargetVec.getNorm();
    var idealHorizontalShotVelocityMps = getIdealHorizontalShotVelocityMps(distanceMeters);

    var robotVelVector = getRobotVelVector();

    var shotVector =
        shooterToTargetVec.times(idealHorizontalShotVelocityMps / distanceMeters).minus(robotVelVector);

    var newHorizontalSpeed = shotVector.getNorm();

    var shooterExitVelocityMps = getShooterExitVelocityMps();
    var ratio = newHorizontalSpeed / shooterExitVelocityMps;
    if (ratio > 1) {
      // We don't have enough shooter velocity
      return;
    }

    var shotAngle = shotVector.getAngle();
    var pitchRad = Math.acos(ratio);

    // TODO: Are we doing a turret?
    hoodIO.runPitch(pitchRad);
  }

  /** Get the 2d vector from the shooter to the target as a {@link Translation2d} */
  private Translation2d getShooterToTargetVec() {
    throw new UnsupportedOperationException("TODO: Implement");
  }

  /** Get the field-relative 2d velocity vector of the shooter as a {@link Translation2d} */
  private Translation2d getRobotVelVector() {
    throw new UnsupportedOperationException("TODO: Implement");
  }

  /**
   * Get the horizontal shot velocity needed to make it into the goal from a certain distance,
   * assuming stationary.
   */
  private double getIdealHorizontalShotVelocityMps(double distanceMeters) {
    throw new UnsupportedOperationException("TODO: Implement");
  }

  /**
   * Get the exit velocity for a projectile that runs through the shooter at the current RPM.
   */
  private double getShooterExitVelocityMps() {
    throw new UnsupportedOperationException("TODO: Implement");
  }
}
