// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BeltFloor extends SubsystemBase {
  private final BeltFloorIO io;
  private final BeltFloorIOInputsAutoLogged inputs = new BeltFloorIOInputsAutoLogged();

  public BeltFloor(BeltFloorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("BeltFloor", inputs);
  }
}
