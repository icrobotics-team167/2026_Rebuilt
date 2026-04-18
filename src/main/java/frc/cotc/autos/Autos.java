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

  private final Swerve swerve;

  public Autos(
      Swerve swerve,
      Shooter shooter,
      Supplier<Command> feedCommand,
      Supplier<Command> intakeCommand) {
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(NONE_NAME, NONE_NAME);
    routines.put(NONE_NAME, Commands::none);

    this.swerve = swerve;
    shootCommand = shooter::sotm;
    this.feedCommand = feedCommand;
    this.intakeCommand = intakeCommand;
    aimCommand = () -> swerve.aimAtTarget(() -> Translation2d.kZero);
    stopCommand = swerve::brake;

    autoFactory =
        new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followPath, true, swerve);
    addRoutine("Center", this::center);
    addRoutine("Center Outpost", this::centerOutpost);
    addRoutine("Center Depot", this::centerDepot);
    addRoutine("Right Trench Mid", this::rightTrenchMid);
    addRoutine("Right Trench Far", this::rightTrenchFar);
    addRoutine("Right Trench Mid Across", this::rightTrenchMidAcross);
    addRoutine("Right Trench Far Across", this::rightTrenchFarAcross);
    addRoutine("Left Trench Mid", this::leftTrenchMid);
    addRoutine("Left Trench Far", this::leftTrenchFar);
    addRoutine("Left Trench Mid Across", this::leftTrenchMidAcross);
    addRoutine("Left Trench Far Across", this::leftTrenchFarAcross);
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
                sequence(trajectory.cmd(), stopCommand.get().withTimeout(1)),
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
    var trajectory2 = ChoreoTraj.CenterOutpost$2.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                swerve
                    .pidToPose(trajectory0.getFinalPose().orElseThrow())
                    .withTimeout(5), // Changed from 3
                trajectory1.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
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
    var trajectory3 = ChoreoTraj.CenterDepot$3.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory3.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightTrenchMid() {
    var routine = autoFactory.newRoutine("Right Trench Mid");
    var trajectory0 = ChoreoTraj.RightTrenchMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchMid$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightTrenchMid$3.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory3.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightTrenchFar() {
    var routine = autoFactory.newRoutine("Right Trench Far");
    var trajectory0 = ChoreoTraj.RightTrenchFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchFar$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightTrenchFar$3.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory3.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightTrenchMidAcross() {
    var routine = autoFactory.newRoutine("Right Trench Mid Across");
    var trajectory0 = ChoreoTraj.RightTrenchMidAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchMidAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchMidAcross$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightTrenchMidAcross$3.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory3.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command rightTrenchFarAcross() {
    var routine = autoFactory.newRoutine("Right Trench Far Across");
    var trajectory0 = ChoreoTraj.RightTrenchFarAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchFarAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchFarAcross$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightTrenchFarAcross$3.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory3.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftTrenchMid() {
    var routine = autoFactory.newRoutine("Left Trench Mid");
    var trajectory0 = ChoreoTraj.LeftTrenchMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchMid$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftTrenchMid$3.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory3.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftTrenchFar() {
    var routine = autoFactory.newRoutine("Left Trench Far");
    var trajectory0 = ChoreoTraj.LeftTrenchFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchFar$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftTrenchFar$3.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory3.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftTrenchMidAcross() {
    var routine = autoFactory.newRoutine("Left Trench Mid Across");
    var trajectory0 = ChoreoTraj.LeftTrenchMidAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchMidAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchMidAcross$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftTrenchMidAcross$3.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory3.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftTrenchFarAcross() {
    var routine = autoFactory.newRoutine("Left Trench Far Across");
    var trajectory0 = ChoreoTraj.LeftTrenchFarAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchFarAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchFarAcross$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftTrenchFarAcross$3.asAutoTraj(routine); // Added

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory1.cmd().deadlineFor(intakeCommand.get()),
                stopCommand.get().withTimeout(5),
                trajectory2.cmd(),
                stopCommand.get().withTimeout(5),
                trajectory3.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }
}
