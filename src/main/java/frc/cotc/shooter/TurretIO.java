// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    double thetaRad;
    double omegaRadPerSec;
    double statorCurrentAmps;
    double supplyCurrentAmps;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void runYaw(double thetaRad, double omegaRadPerSec) {
    Units.radiansToRotations(thetaRad);
    Units.radiansToRotations(omegaRadPerSec);
  }

  public default void run() {}

  public default void stop() {}
}
