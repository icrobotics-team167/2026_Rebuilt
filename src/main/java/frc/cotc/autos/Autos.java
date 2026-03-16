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

  private final Swerve swerve;
  private final Supplier<Command> shootCommand, feedCommand, intakeCommand, aimCommand, stopCommand;

  public Autos(
      Swerve swerve, Shooter shooter, Supplier<Command> feedCommand, IntakeRoller intakeRoller) {
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(NONE_NAME, NONE_NAME);
    routines.put(NONE_NAME, Commands::none);

    this.swerve = swerve;
    shootCommand = shooter::sotm;
    this.feedCommand = feedCommand;
    intakeCommand = intakeRoller::intake;
    aimCommand = () -> swerve.aimAtTarget(() -> Translation2d.kZero);
    stopCommand = swerve::brake;

    autoFactory =
        new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followPath, true, swerve);
    addRoutine("Center", this::center);
    addRoutine("Right Bump Mid", this::rightBumpMid);
    addRoutine("Left Bump Mid", this::leftBumpMid);
    addRoutine("Far Right Outpost", this::farRightOutpost);
    addRoutine("Far Left Depot", this::farLeftDepot);
    addRoutine("Right Bump Mid Outpost", this::rightBumpMidOutpost);
    addRoutine("Left Bump Mid Depot", this::leftBumpMidDepot);
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
}
