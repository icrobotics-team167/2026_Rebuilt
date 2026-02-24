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

public class TurretFeeder extends SubsystemBase {
  private final TurretFeederIO io;
  private final TurretFeederIOInputsAutoLogged inputs = new TurretFeederIOInputsAutoLogged();

  public TurretFeeder(TurretFeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TurretFeeder", inputs);
  }

  public Command runFeeder() {
    return run(io::run).finallyDo(io::stop);
  }
}
