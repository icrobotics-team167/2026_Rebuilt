// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.cotc.Robot;
import frc.cotc.intake.IntakeRoller;
import frc.cotc.shooter.Shooter;
import frc.cotc.swerve.Swerve;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final AutoFactory autoFactory;
  private final LoggedDashboardChooser<String> chooser;
  private final HashMap<String, Supplier<Command>> routines = new HashMap<>();
  private final String NONE_NAME = "Do Nothing";

  private final Supplier<Command> shootCommand, feedCommand, intakeCommand, aimCommand, stopCommand;

  public Autos(
      Swerve swerve, Shooter shooter, Supplier<Command> feedCommand, IntakeRoller intakeRoller) {
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(NONE_NAME, NONE_NAME);
    routines.put(NONE_NAME, Commands::none);

    shootCommand = shooter::sotm;
    this.feedCommand = feedCommand;
    intakeCommand = intakeRoller::intake;
    aimCommand = () -> swerve.aimAtTarget(() -> Translation2d.kZero);
    stopCommand = swerve::brake;

    autoFactory =
        new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followPath, true, swerve);
    addRoutine("Center", this::center);
    addRoutine("Center Outpost", this::centerOutpost);
    addRoutine("Center Depot", this::centerDepot);
    addRoutine("Center Outpost Mid", this::centerOutpostMid);
    addRoutine("Center Depot Mid", this::centerDepotMid);
    addRoutine("Center Outpost Far", this::centerOutpostFar);
    addRoutine("Center Depot Far", this::centerDepotFar);
    addRoutine("Center Outpost Close", this::centerOutpostClose);
    addRoutine("Center Depot Close", this::centerDepotClose);
    addRoutine("Far Right Outpost", this::farRightOutpost);
    addRoutine("Far Left Depot", this::farLeftDepot);
    addRoutine("Far Right Outpost Mid", this::farRightOutpostMid);
    addRoutine("Far Left Depot Mid", this::farLeftDepotMid);
    addRoutine("Far Right Outpost Far", this::farRightOutpostFar);
    addRoutine("Far Left Depot Far", this::farLeftDepotFar);
    addRoutine("Far Right Outpost Close", this::farRightOutpostClose);
    addRoutine("Far Left Depot Close", this::farLeftDepotClose);
    addRoutine("Right Bump Mid", this::rightBumpMid);
    addRoutine("Left Bump Mid", this::leftBumpMid);
    addRoutine("Right Bump Mid Outpost", this::rightBumpMidOutpost);
    addRoutine("Left Bump Mid Depot", this::leftBumpMidDepot);
    addRoutine("Right Bump Far", this::rightBumpFar);
    addRoutine("Left Bump Far", this::leftBumpFar);
    addRoutine("Right Bump Far Outpost", this::rightBumpFarOutpost);
    addRoutine("Left Bump Far Depot", this::leftBumpFarDepot);
    addRoutine("Right Bump Close", this::rightBumpClose);
    addRoutine("Left Bump Close", this::leftBumpClose);
    addRoutine("Right Bump Mid Close", this::rightBumpMidClose);
    addRoutine("Left Bump Mid Close", this::leftBumpMidClose);
    addRoutine("Right Bump Far Close", this::rightBumpFarClose);
    addRoutine("Left Bump Far Close", this::leftBumpFarClose);
    addRoutine("Right Bump Far Across", this::rightBumpFarAcross);
    addRoutine("Left Bump Far Across", this::leftBumpFarAcross);
    addRoutine("Right Bump Far Across Close", this::rightBumpFarAcrossClose);
    addRoutine("Left Bump Far Across Close", this::leftBumpFarAcrossClose);
    addRoutine("Right Bump Far Across Depot", this::rightBumpFarAcrossDepot);
    addRoutine("Left Bump Far Across Outpost", this::leftBumpFarAcrossOutpost);
  }

  private String selectedCommandName = NONE_NAME;
  private Command selectedCommand = none();
  private boolean selectedOnRed = false;

  private final Alert selectedNonexistentAuto =
      new Alert("Selected an auto that isn't an option!", Alert.AlertType.kError);
  private final Alert loadedAutoAlert = new Alert("", Alert.AlertType.kInfo);

  public void update() {
    if (DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
      var selected = chooser.get();
      if (selected.equals(selectedCommandName) && selectedOnRed == Robot.isOnRed()) {
        return;
      }
      if (!routines.containsKey(selected)) {
        selected = NONE_NAME;
        selectedNonexistentAuto.set(true);
      } else {
        selectedNonexistentAuto.set(false);
      }
      selectedCommandName = selected;
      selectedCommand = routines.get(selected).get().withName(selectedCommandName);
      selectedOnRed = Robot.isOnRed();
      loadedAutoAlert.setText("Loaded Auto: " + selectedCommandName);
      loadedAutoAlert.set(true);
    }
  }

  public void clear() {
    selectedCommandName = NONE_NAME;
    selectedCommand = none();
    selectedOnRed = false;
  }

  public Command warmup() {
    System.out.println("Warmup command instantiated");
    return sequence(
            print("Warming up"), autoFactory.trajectoryCmd("Warmup"), print("Warmup complete"))
        .ignoringDisable(true);
  }

  public Command getSelectedCommand() {
    return selectedCommand;
  }

  private void addRoutine(String name, Supplier<Command> generator) {
    chooser.addOption(name, name);
    routines.put(name, generator);
  }

  private Command center() {
    var routine = autoFactory.newRoutine("Center");
    var trajectory = ChoreoTraj.Center.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory.resetOdometry(),
                trajectory.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command centerOutpost() {
    var routine = autoFactory.newRoutine("Center Outpost");
    var trajectory0 = ChoreoTraj.CenterOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.CenterOutpost$1.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory1.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command centerDepot() {
    var routine = autoFactory.newRoutine("Center Depot");
    var trajectory0 = ChoreoTraj.CenterDepot$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.CenterDepot$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.CenterDepot$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command centerOutpostMid() {
    var routine = autoFactory.newRoutine("Center Outpost Mid");
    var trajectory0 = ChoreoTraj.CenterOutpostMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.CenterOutpostMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.CenterOutpostMid$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.CenterOutpostMid$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.CenterOutpostMid$4.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory1.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory2.cmd(),
                trajectory3.cmd().deadlineFor(intakeCommand.get()),
                trajectory4.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command centerDepotMid() {
    var routine = autoFactory.newRoutine("Center Depot Mid");
    var trajectory0 = ChoreoTraj.CenterDepotMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.CenterDepotMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.CenterDepotMid$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.CenterDepotMid$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.CenterDepotMid$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.CenterDepotMid$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command centerOutpostFar() {
    var routine = autoFactory.newRoutine("Center Outpost Far");
    var trajectory0 = ChoreoTraj.CenterOutpostFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.CenterOutpostFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.CenterOutpostFar$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.CenterOutpostFar$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.CenterOutpostFar$4.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory1.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory2.cmd(),
                trajectory3.cmd().deadlineFor(intakeCommand.get()),
                trajectory4.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command centerDepotFar() {
    var routine = autoFactory.newRoutine("Center Depot Far");
    var trajectory0 = ChoreoTraj.CenterDepotFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.CenterDepotFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.CenterDepotFar$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.CenterDepotFar$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.CenterDepotFar$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.CenterDepotFar$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command centerOutpostClose() {
    var routine = autoFactory.newRoutine("Center Outpost Close");
    var trajectory0 = ChoreoTraj.CenterOutpostClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.CenterOutpostClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.CenterOutpostClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.CenterOutpostClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.CenterOutpostClose$4.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory1.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory2.cmd(),
                trajectory3.cmd().deadlineFor(intakeCommand.get()),
                trajectory4.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command centerDepotClose() {
    var routine = autoFactory.newRoutine("Center Depot Close");
    var trajectory0 = ChoreoTraj.CenterDepotClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.CenterDepotClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.CenterDepotClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.CenterDepotClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.CenterDepotClose$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.CenterDepotClose$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command farRightOutpost() {
    var routine = autoFactory.newRoutine("Far Right Outpost");
    var trajectory0 = ChoreoTraj.FarRightOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.FarRightOutpost$1.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory1.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command farLeftDepot() {
    var routine = autoFactory.newRoutine("Far Left Depot");
    var trajectory0 = ChoreoTraj.FarLeftDepot$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.FarLeftDepot$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.FarLeftDepot$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command farRightOutpostMid() {
    var routine = autoFactory.newRoutine("Far Right Outpost Mid");
    var trajectory0 = ChoreoTraj.FarRightOutpostMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.FarRightOutpostMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.FarRightOutpostMid$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.FarRightOutpostMid$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.FarRightOutpostMid$4.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory1.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory2.cmd(),
                trajectory3.cmd().deadlineFor(intakeCommand.get()),
                trajectory4.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command farLeftDepotMid() {
    var routine = autoFactory.newRoutine("Far Left Depot Mid");
    var trajectory0 = ChoreoTraj.FarLeftDepotMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.FarLeftDepotMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.FarLeftDepotMid$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.FarLeftDepotMid$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.FarLeftDepotMid$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.FarLeftDepotMid$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command farRightOutpostClose() {
    var routine = autoFactory.newRoutine("Far Right Outpost Close");
    var trajectory0 = ChoreoTraj.FarRightOutpostClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.FarRightOutpostClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.FarRightOutpostClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.FarRightOutpostClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.FarRightOutpostClose$4.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory1.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory2.cmd(),
                trajectory3.cmd().deadlineFor(intakeCommand.get()),
                trajectory4.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command farLeftDepotClose() {
    var routine = autoFactory.newRoutine("Far Left Depot Close");
    var trajectory0 = ChoreoTraj.FarLeftDepotClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.FarLeftDepotClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.FarLeftDepotClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.FarLeftDepotClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.FarLeftDepotClose$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.FarLeftDepotClose$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command farRightOutpostFar() {
    var routine = autoFactory.newRoutine("Far Right Outpost Far");
    var trajectory0 = ChoreoTraj.FarRightOutpostFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.FarRightOutpostFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.FarRightOutpostFar$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.FarRightOutpostFar$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.FarRightOutpostFar$4.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory1.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory2.cmd(),
                trajectory3.cmd().deadlineFor(intakeCommand.get()),
                trajectory4.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command farLeftDepotFar() {
    var routine = autoFactory.newRoutine("Far Left Depot Far");
    var trajectory0 = ChoreoTraj.FarLeftDepotFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.FarLeftDepotFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.FarLeftDepotFar$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.FarLeftDepotFar$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.FarLeftDepotFar$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.FarLeftDepotFar$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpMid() {
    var routine = autoFactory.newRoutine("Right Bump Mid");
    var trajectory0 = ChoreoTraj.RightBumpMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpMid$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpMid() {
    var routine = autoFactory.newRoutine("Left Bump Mid");
    var trajectory0 = ChoreoTraj.LeftBumpMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpMid$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpMidOutpost() {
    var routine = autoFactory.newRoutine("Right Bump Mid Outpost");
    var trajectory0 = ChoreoTraj.RightBumpMidOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpMidOutpost$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpMidOutpost$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightBumpMidOutpost$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.RightBumpMidOutpost$4.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory4.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpMidDepot() {
    var routine = autoFactory.newRoutine("Left Bump Mid Depot");
    var trajectory0 = ChoreoTraj.LeftBumpMidDepot$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpMidDepot$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpMidDepot$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftBumpMidDepot$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftBumpMidDepot$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.LeftBumpMidDepot$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpFar() {
    var routine = autoFactory.newRoutine("Right Bump Far");
    var trajectory0 = ChoreoTraj.RightBumpFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpFar$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpFar() {
    var routine = autoFactory.newRoutine("Left Bump Far");
    var trajectory0 = ChoreoTraj.LeftBumpFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpFar$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpFarOutpost() {
    var routine = autoFactory.newRoutine("Right Bump Far Outpost");
    var trajectory0 = ChoreoTraj.RightBumpFarOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpFarOutpost$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpFarOutpost$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightBumpFarOutpost$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.RightBumpFarOutpost$4.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory4.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpFarDepot() {
    var routine = autoFactory.newRoutine("Left Bump Far Depot");
    var trajectory0 = ChoreoTraj.LeftBumpFarDepot$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpFarDepot$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpFarDepot$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftBumpFarDepot$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftBumpFarDepot$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.LeftBumpFarDepot$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpClose() {
    var routine = autoFactory.newRoutine("Right Bump Close");
    var trajectory0 = ChoreoTraj.RightBumpClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpClose$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpClose() {
    var routine = autoFactory.newRoutine("Left Bump Close");
    var trajectory0 = ChoreoTraj.LeftBumpClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpClose$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpMidClose() {
    var routine = autoFactory.newRoutine("Right Bump Mid Close");
    var trajectory0 = ChoreoTraj.RightBumpMidClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpMidClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpMidClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightBumpMidClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.RightBumpMidClose$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.RightBumpMidClose$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpMidClose() {
    var routine = autoFactory.newRoutine("Left Bump Mid Close");
    var trajectory0 = ChoreoTraj.LeftBumpMidClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpMidClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpMidClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftBumpMidClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftBumpMidClose$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.LeftBumpMidClose$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpFarClose() {
    var routine = autoFactory.newRoutine("Right Bump Far Close");
    var trajectory0 = ChoreoTraj.RightBumpFarClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpFarClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpFarClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightBumpFarClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.RightBumpFarClose$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.RightBumpFarClose$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpFarClose() {
    var routine = autoFactory.newRoutine("Left Bump Far Close");
    var trajectory0 = ChoreoTraj.LeftBumpFarClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpFarClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpFarClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftBumpFarClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftBumpFarClose$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.LeftBumpFarClose$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpFarAcross() {
    var routine = autoFactory.newRoutine("Right Bump Far Across");
    var trajectory0 = ChoreoTraj.RightBumpFarAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpFarAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpFarAcross$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpFarAcross() {
    var routine = autoFactory.newRoutine("Left Bump Far Across");
    var trajectory0 = ChoreoTraj.LeftBumpFarAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpFarAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpFarAcross$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpFarAcrossClose() {
    var routine = autoFactory.newRoutine("Right Bump Far Across Close");
    var trajectory0 = ChoreoTraj.RightBumpFarAcrossClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpFarAcrossClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpFarAcrossClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightBumpFarAcrossClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.RightBumpFarAcrossClose$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.RightBumpFarAcrossClose$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpFarAcrossClose() {
    var routine = autoFactory.newRoutine("Left Bump Far Across Close");
    var trajectory0 = ChoreoTraj.LeftBumpFarAcrossClose$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpFarAcrossClose$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpFarAcrossClose$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftBumpFarAcrossClose$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftBumpFarAcrossClose$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.LeftBumpFarAcrossClose$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightBumpFarAcrossDepot() {
    var routine = autoFactory.newRoutine("Right Bump Far Across Depot");
    var trajectory0 = ChoreoTraj.RightBumpFarAcrossDepot$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpFarAcrossDepot$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpFarAcrossDepot$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightBumpFarAcrossDepot$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.RightBumpFarAcrossDepot$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.RightBumpFarAcrossDepot$5.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftBumpFarAcrossOutpost() {
    var routine = autoFactory.newRoutine("Left Bump Far Across Outpost");
    var trajectory0 = ChoreoTraj.LeftBumpFarAcrossOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpFarAcrossOutpost$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpFarAcrossOutpost$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftBumpFarAcrossOutpost$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftBumpFarAcrossOutpost$4.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                trajectory2.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(5),
                trajectory3.cmd(),
                stopCommand.get().withTimeout(3),
                trajectory4.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }
}
