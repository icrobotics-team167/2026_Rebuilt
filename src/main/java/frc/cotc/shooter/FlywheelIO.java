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
    double projectileVelMetersPerSec;
    double motor0statorCurrentAmps;
    double motor1statorCurrentAmps;
    double motor0SupplyCurrentAmps;
    double motor1SupplyCurrentAmps;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void runVel(double projectileVelMetersPerSec) {}
}
