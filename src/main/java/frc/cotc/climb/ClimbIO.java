// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  class ClimbIOInputs {
    double posRad;
    // implement later?
    double statorCurrentAmps;
    double supplyCurrentAmps;
    boolean isAtTop; // implement a limit switch
    boolean isAtBottom; // this too
  }

  default void updateInputs(ClimbIOInputs inputs) {}

  default void stop() {}

  default void deploy() {}

  default void retract() {}

  default void climb() {}
}
