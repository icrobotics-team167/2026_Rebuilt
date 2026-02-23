// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface BeltFloorIO {
  public default void updateInputs(BeltFloorIOInputs inputs) {}

  public default void run() {}

  public default void stop() {}

  @AutoLog
  public class BeltFloorIOInputs {
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }
}
