// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.feeder;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class Feeder {
  private final BeltFloor beltFloor;
  private final Raceway raceway;
  private final TurretFeeder turretFeeder;

  public Feeder(BeltFloorIO beltFloorIO, RacewayIO racewayIO, TurretFeederIO turretFeederIO) {
    beltFloor = new BeltFloor(beltFloorIO);
    raceway = new Raceway(racewayIO);
    turretFeeder = new TurretFeeder(turretFeederIO);
  }

  boolean feeding = false;

  public Command feed() {
    var delaySeconds = 0.5;
    return sequence(
            runOnce(() -> feeding = true),
            new ScheduleCommand(
                parallel(
                        turretFeeder
                            .runFeeder()
                            .withDeadline(
                                waitUntil(() -> !feeding)
                                    .withName("Wait until bumper release")
                                    .andThen(
                                        waitSeconds(delaySeconds * 2).withName("Delay shutdown")))
                            .withName("Start up/Shut down feeder"),
                        waitSeconds(delaySeconds)
                            .withName("Delay startup")
                            .andThen(raceway.runRaceway())
                            .withDeadline(
                                waitUntil(() -> !feeding)
                                    .withName("Wait until bumper release")
                                    .andThen(waitSeconds(delaySeconds).withName("Delay shutdown")))
                            .withName("Start up/Shut down raceway"),
                        waitSeconds(delaySeconds * 2)
                            .withName("Delay startup")
                            .andThen(beltFloor.runBelt())
                            .onlyWhile(() -> feeding)
                            .withName("Start up/Shut down belt"))
                    .withName("Start up/Shut down feed system")),
            idle())
        .finallyDo(() -> feeding = false)
        .withName("Feed");
  }
}
