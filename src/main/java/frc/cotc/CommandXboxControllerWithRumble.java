// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An extension of {@link CommandXboxController} that adds a Command to rumble the controller. */
public class CommandXboxControllerWithRumble extends CommandXboxController {
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandXboxControllerWithRumble(int port) {
    super(port);
  }

  public Command rumble(double timeSeconds) {
    return run(() -> setRumble(GenericHID.RumbleType.kBothRumble, 1))
        .withTimeout(timeSeconds)
        .finallyDo(() -> setRumble(GenericHID.RumbleType.kBothRumble, 0))
        .withName("Rumble");
  }
}
