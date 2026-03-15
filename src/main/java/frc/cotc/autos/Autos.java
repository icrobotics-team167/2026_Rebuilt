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
  private final Supplier<Command> feedCommand, intakeCommand, aimCommand, stopCommand;

  public Autos(
      Swerve swerve, Shooter shooter, Supplier<Command> feedCommand, IntakeRoller intakeRoller) {
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(NONE_NAME, NONE_NAME);
    routines.put(NONE_NAME, Commands::none);

    this.swerve = swerve;
    this.feedCommand = feedCommand;
    intakeCommand = intakeRoller::intake;
    aimCommand = () -> swerve.aimAtTarget(() -> Translation2d.kZero);
    stopCommand = swerve::brake;

    autoFactory =
        new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followPath, true, swerve);

    addRoutine("Depot Left", this::depotLeft);
    addRoutine("Outpost Right", this::outpostRight);
    addRoutine("Center", this::center);
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

  private Command depotLeft() {
    var routine = autoFactory.newRoutine("Depot Left");
    var segment0 = ChoreoTraj.DepotLeft$0.asAutoTraj(routine);
    var segment1 = ChoreoTraj.DepotLeft$1.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                segment0.resetOdometry(),
                segment0.cmd().deadlineFor(intakeCommand.get().asProxy()),
                segment1.cmd(),
                parallel(
                    aimCommand.get(),
                    waitSeconds(1).withName("Delay").andThen(feedCommand.get().asProxy()))));

    return routine.cmd();
  }

  private Command outpostRight() {
    var routine = autoFactory.newRoutine("Outpost Right");
    var segment0 = ChoreoTraj.OutpostRight$0.asAutoTraj(routine);
    var segment1 = ChoreoTraj.OutpostRight$1.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                segment0.resetOdometry(),
                segment0.cmd(),
                swerve.brake().withTimeout(5),
                segment1.cmd(),
                parallel(
                    aimCommand.get(),
                    waitSeconds(1).withName("Delay").andThen(feedCommand.get().asProxy()))));

    return routine.cmd();
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
                    waitSeconds(1).withName("Delay").andThen(feedCommand.get().asProxy()))));

    return routine.cmd();
  }
}
