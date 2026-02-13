// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void run(double thetaRad) {}

  public default void stop() {}

  @AutoLog
  public class IntakePivotIOInputs {
    public double appliedVolts1 = 0.0;
    public double statorCurrentAmps1 = 0.0;
    public double supplyCurrentAmps1 = 0.0;
    public double appliedVolts2 = 0.0;
    public double statorCurrentAmps2 = 0.0;
    public double supplyCurrentAmps2 = 0.0;
    public double thetaRad = 0.0;
  }
}
