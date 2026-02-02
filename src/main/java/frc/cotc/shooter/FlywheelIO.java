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
  static class FlywheelIOInputs {
    double projectileVelMetersPerSec = 0.0;
    double appliedVolts = 0.0;

    double motor0StatorCurrentAmps = 0.0;
    double motor1StatorCurrentAmps = 0.0;
    double motor2StatorCurrentAmps = 0.0;
    double motor3StatorCurrentAmps = 0.0;

    double motor0SupplyCurrentAmps = 0.0;
    double motor1SupplyCurrentAmps = 0.0;
    double motor2SupplyCurrentAmps = 0.0;
    double motor3SupplyCurrentAmps = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void runVel(double projectileVelMetersPerSec) {}

  default void stop() {}
}
