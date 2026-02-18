// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Raceway extends SubsystemBase {
  private final RacewayIO io;
  private final RacewayIOInputsAutoLogged inputs = new RacewayIOInputsAutoLogged();

  public Raceway(RacewayIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Raceway", inputs);
  }

  public Command runRaceway() {
    return run(io::run).finallyDo(io::stop);
  }
}
