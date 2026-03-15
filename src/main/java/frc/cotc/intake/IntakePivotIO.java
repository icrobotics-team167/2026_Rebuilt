// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  default void updateInputs(IntakePivotIOInputs inputs) {}

  default void run(double volts) {}

  default void stop() {
    run(0.0);
  }

  @AutoLog
  class IntakePivotIOInputs {
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double velocityRotPerSec = 0.0;
    public double pivotAngleRad = 0.0;
    public double appliedVolts = 0.0;
  }
}
