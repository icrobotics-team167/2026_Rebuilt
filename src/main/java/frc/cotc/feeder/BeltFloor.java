// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.feeder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BeltFloor extends SubsystemBase {
  private final BeltFloorIO io;
  private final BeltFloorIOInputsAutoLogged inputs = new BeltFloorIOInputsAutoLogged();
  private final double DEFAULT_JAM_TIME = 0.2;
  private final Debouncer debouncer;
  private final double JAM_VOLTAGE = 40.0;
  private final double JAM_SPEED = 40.0;

  public BeltFloor(BeltFloorIO io) {
    this.io = io;
    debouncer = new Debouncer(DEFAULT_JAM_TIME, Debouncer.DebounceType.kBoth);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("BeltFloor", inputs);
  }

  public Command runIntermittenly() {
    double interval = 1.0; // placeholder

    return Commands.repeatingSequence(
            runBelt().withTimeout(interval), stopBelt().withTimeout(interval))
        .withName("RunIntermittenly");
  }

  public Command runBelt() {
    return run(io::run);
  }

  public Command stopBelt() {
    return run(io::stop);
  }

  public boolean isJam() {
    return debouncer.calculate(
        inputs.motorSpeed < JAM_SPEED && inputs.statorCurrentAmps > JAM_VOLTAGE);
  }
}
