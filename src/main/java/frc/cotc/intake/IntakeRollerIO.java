// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  default void updateInputs(IntakeRollerIOInputs inputs) {}

  default void run() {}

  default void runReverse() {}

  default void stop() {}

  @AutoLog
  class IntakeRollerIOInputs {
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }
}
