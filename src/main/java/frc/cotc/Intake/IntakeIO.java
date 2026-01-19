// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void run() {}

  public default void runReverse() {}

  public default void stop() {}

  @AutoLog
  public class IntakeIOInputs {
    public double appleidVolts1 = 0.0;
    public double statorCurrentAmps1 = 0.0;
    public double supplyCurrentAmps1 = 0.0;
    public double appleidVolts2 = 0.0;
    public double statorCurrentAmps2 = 0.0;
    public double supplyCurrentAmps2 = 0.0;
  }
}
