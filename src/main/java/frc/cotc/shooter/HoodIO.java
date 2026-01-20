// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    public double pitchRad;
    public double pitchVelRadPerSec;
    public double motorStatorCurrentAmps;
    public double motorSupplyCurrentAmps;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void runPitch(double pitchRad) {}
}
