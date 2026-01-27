// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public double projectileVelMetersPerSec;
    public double appliedVolts;

    // Left Side
    public double motor0StatorCurrentAmps;
    public double motor1StatorCurrentAmps;
    public double motor0SupplyCurrentAmps;
    public double motor1SupplyCurrentAmps;

    // Right Side
    public double motor2StatorCurrentAmps;
    public double motor3StatorCurrentAmps;
    public double motor2SupplyCurrentAmps;
    public double motor3SupplyCurrentAmps;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  // Run the shooter at a target Projectile Velocity.
  default void runVel(double projectileVelMetersPerSec) {}

  default void stop() {}

  default void setGains(double kP, double kV) {}

  default void addCalibrationPoint(double mps, double rps) {}
}
