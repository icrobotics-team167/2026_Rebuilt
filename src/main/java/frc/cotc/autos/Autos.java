// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.cotc.Robot;
import frc.cotc.shooter.Shooter;
import frc.cotc.swerve.Swerve;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * A class to control the autonomous routines.
 *
 * <p>Based on {@link AutoChooser}, modified to use AdvantageKit's {@link LoggedDashboardChooser}
 */
public class Autos {
  /** A factory to generate {@link AutoRoutine}s for */
  private final AutoFactory autoFactory;

  /**
   * A replay-compatible version of {@link SendableChooser} to display a dropdown menu on the driver
   * dashboard.
   */
  private final LoggedDashboardChooser<String> chooser;

  /**
   * A map of auto routines to their corresponding Command suppliers.
   *
   * <p>Suppliers to construct on select are used instead of constructing on boot to avoid long boot
   * times.
   */
  private final HashMap<String, Supplier<Command>> routines = new HashMap<>();

  /** The name of the default auto routine, which does nothing. */
  private final String NONE_NAME = "Do Nothing";

  /**
   * Suppliers for Commands that are used in the auto routines, such as shooting, feeding, and
   * aiming.
   */
  private final Supplier<Command> shootCommand, feedCommand, intakeCommand, aimCommand, stopCommand;

  /** The {@link Swerve} subsystem */
  private final Swerve swerve;

  public Autos(
      Swerve swerve,
      Shooter shooter,
      Supplier<Command> feedCommand,
      Supplier<Command> intakeCommand) {
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(NONE_NAME, NONE_NAME); // Default is to do nothing, safest option
    routines.put(NONE_NAME, Commands::none); // Add the "do nothing" auto to the map

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
    addRoutine("Right Bump Mid", this::rightBumpMid);
    addRoutine("Right Bump Mid Across", this::rightBumpMidAcross);
    addRoutine("Right Bump Far", this::rightBumpFar);
    addRoutine("Right Bump Far Across", this::rightBumpFarAcross);
    addRoutine("Right Trench Mid", this::rightTrenchMid);
    addRoutine("Right Trench Mid Outpost", this::RightTrenchMidOutpost);
    addRoutine("Right Trench Far", this::rightTrenchFar);
    addRoutine("Right Trench Far Outpost", this::RightTrenchFarOutpost);
    addRoutine("Right Trench Mid Across", this::rightTrenchMidAcross);
    addRoutine("Right Trench Mid Across Depot", this::RightTrenchMidAcrossDepot);
    addRoutine("Right Trench Far Across", this::rightTrenchFarAcross);
    addRoutine("Left Bump Mid", this::leftBumpMid);
    addRoutine("Left Bump Mid Across", this::leftBumpMidAcross);
    addRoutine("Left Bump Far", this::leftBumpFar);
    addRoutine("Left Bump Far Across", this::leftBumpFarAcross);
    addRoutine("Left Trench Mid", this::leftTrenchMid);
    addRoutine("Left Trench Far", this::leftTrenchFar);
    addRoutine("Left Trench Far Across Outpost", this::leftTrenchFarAcrossOutpost);
    addRoutine("Left Trench Far Depot", this::leftTrenchFarDepot);
    addRoutine("Left Trench Mid Across", this::leftTrenchMidAcross);
    addRoutine("Left Trench Mid Across Outpost", this::leftTrenchMidAcrossOutpost);
    addRoutine("Left Trench Mid Depot", this::leftTrenchMidDepot);
    addRoutine("Left Trench Far Across", this::leftTrenchFarAcross);
  }

  /** The currently selected auto. */
  private String selectedCommandName = NONE_NAME;

  /**
   * The Command for the currently loaded auto. This is the Command that will be run when the robot
   * is enabled.
   */
  private Command selectedCommand = none();

  /**
   * A boolean indicating whether the currently loaded auto is flipped to be on the red alliance.
   */
  private boolean selectedOnRed = false;

  /**
   * An error {@link Alert} that is displayed when the selected auto is not found in the routines
   * map.
   */
  private final Alert selectedNonexistentAuto =
      new Alert("Selected an auto that isn't an option!", Alert.AlertType.kError);

  /** An info {@link Alert} that is displayed when the currently loaded auto is changed. */
  private final Alert loadedAutoAlert = new Alert("", Alert.AlertType.kInfo);

  /**
   * Updates the currently selected auto. If the currently loaded auto and the currently selected
   * auto are the same, no-op.
   *
   * <p>Autos are lazy-loaded using {@link Supplier}s to avoid long boot times, but are loaded
   * before match start to avoid a delay when the auto is actually run.
   */
  public void update() {
    // Only update if we are confirmed to be connected to the driver station
    if (DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
      // If the dashboard-selected auto is the same as the currently loaded auto, no-op
      var selected = chooser.get();
      if (selected.equals(selectedCommandName) && selectedOnRed == Robot.isOnRed()) {
        return;
      }
      // Check if the selected auto exists. If not, fall back to the "do nothing" and show an alert
      if (!routines.containsKey(selected)) {
        selected = NONE_NAME;
        selectedNonexistentAuto.set(true);
      } else {
        selectedNonexistentAuto.set(false);
      }
      // Load the selected auto
      selectedCommandName = selected;
      selectedCommand = routines.get(selected).get().withName(selectedCommandName);
      selectedOnRed = Robot.isOnRed();
      loadedAutoAlert.setText("Loaded Auto: " + selectedCommandName);
      loadedAutoAlert.set(true);
    }
  }

  /**
   * Reset* the currently selected auto back to the "do nothing" auto.
   *
   * <p>However, due to an oversight this doesn't actually work: since there's no way to affect the
   * dashboard's selection from robot code, and therefore the next call of {@link #update()} will
   * load the dashboard-selected auto again.
   */
  public void clear() {
    selectedCommandName = NONE_NAME;
    selectedCommand = none();
    selectedOnRed = false;
  }

  /**
   * Returns a Command that runs all the necessary code to run an auto, without actually moving
   * since the robot is disabled when this runs. This forces the JVM to load all the necessary
   * classes and code ahead of time, so that when the robot is enabled, the auto runs without a
   * delay.
   */
  public Command warmup() {
    System.out.println("Warmup command instantiated");
    return sequence(
            print("Warming up"),
            autoFactory.trajectoryCmd("Warmup"),
            stopCommand.get().withTimeout(0.1),
            print("Warmup complete"))
        .ignoringDisable(true);
  }

  /**
   * Gets the currently selected auto.
   *
   * @return The currently selected auto.
   */
  public Command getSelectedCommand() {
    return selectedCommand;
  }

  /**
   * Adds a routine to the auto chooser and the routines map.
   *
   * @param name The name of the routine to display in the chooser.
   * @param generator The Supplier that generates the Command for the routine.
   */
  private void addRoutine(String name, Supplier<Command> generator) {
    chooser.addOption(name, name);
    routines.put(name, generator);
  }

  // The methods to return the auto commands.
  // I probably should have come up with a way to programmatically generate these different
  // combinations, but all of these were done manually...

  private Command center() {
    var routine = autoFactory.newRoutine("Center");
    var trajectory = ChoreoTraj.Center.asAutoTraj(routine);

    // The Choreo auto API is designed around highly dynamic branching autos, so it feels like a
    // waste/unnecessary complexity to use it for a fixed sequence auto, but I used it anyways
    // cause I like it better than the PathPlanner API, despite feature parity.
    // A good example of what the API is truly capable of is demonstrated below:
    // https://github.com/icrobotics-team167/2024_Off_Season/blob/main/src/main/java/frc/cotc/Autos.java#L59
    routine
        .active()
        .onTrue(
            sequence(
                // Reset the odometry to the starting position
                // Consistency is important, and we can assume the robot is starting at the same
                // physical position every time, so we can reset the odometry to that position
                trajectory.resetOdometry(),
                sequence(
                    trajectory.cmd(),
                    // Stop the robot for 1 sec to let the momentum settle before aiming
                    // Usually the waitSeconds(1) can get that done but not in this case
                    stopCommand.get().withTimeout(1)),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  // Outpost autos went unused for the most part after we ended up putting a net over the top of
  // the hopper, preventing a top load.
  private Command centerOutpost() {
    var routine = autoFactory.newRoutine("Center Outpost");
    var trajectory0 = ChoreoTraj.CenterOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.CenterOutpost$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.CenterOutpost$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(),
                // Simple PID loop to go to the final position of the trajectory to ensure we are
                // in the right position to load from the outpost
                swerve.pidToPose(trajectory0.getFinalPose().orElseThrow()).withTimeout(5),
                trajectory1.cmd(),
                trajectory2.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  // Depot autos were rough because the bump around the depot area messed up odometry hard, and
  // our cameras were not good enough to compensate.
  // I ended up just extending the trajectory to clip way into the wall to account for the fact
  // that the odometry would be off.
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

  // The mid autos line up such that the bumper goes right up against the centerline, but doesn't
  // cross it to the other side.
  private Command rightTrenchMid() {
    var routine = autoFactory.newRoutine("Right Trench Mid");
    var trajectory0 = ChoreoTraj.RightTrenchMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchMid$2.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trajectory0.resetOdometry(),
                trajectory0.cmd(), // Go from start to center under the trench
                trajectory1.cmd().deadlineFor(intakeCommand.get()), // Sweep across and intake
                trajectory2.cmd(), // Cross the bump
                parallel( // Shoot
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  // The far autos line up such that the robot partially, but not fully, crosses the centerline to
  // the other side.
  // This disrupts opponent autos far more but was risky and never used because it only works if
  // the opposing side doesn't go for a centerline auto and everyone did.
  private Command rightTrenchFar() {
    var routine = autoFactory.newRoutine("Right Trench Far");
    var trajectory0 = ChoreoTraj.RightTrenchFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchFar$2.asAutoTraj(routine);

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

  // The across autos sweeped all the way across instead of staying on one half of the field,
  // gathering more fuel and disrupting autos more. Intended for when our alliance partners
  // didn't run a centerline auto, but again, everyone did.
  private Command rightTrenchMidAcross() {
    var routine = autoFactory.newRoutine("Right Trench Mid Across");
    var trajectory0 = ChoreoTraj.RightTrenchMidAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchMidAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchMidAcross$2.asAutoTraj(routine);

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

  private Command rightTrenchFarAcross() {
    var routine = autoFactory.newRoutine("Right Trench Far Across");
    var trajectory0 = ChoreoTraj.RightTrenchFarAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchFarAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchFarAcross$2.asAutoTraj(routine);

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

  private Command leftTrenchMid() {
    var routine = autoFactory.newRoutine("Left Trench Mid");
    var trajectory0 = ChoreoTraj.LeftTrenchMid$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchMid$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchMid$2.asAutoTraj(routine);

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

  private Command leftTrenchFar() {
    var routine = autoFactory.newRoutine("Left Trench Far");
    var trajectory0 = ChoreoTraj.LeftTrenchFar$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchFar$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchFar$2.asAutoTraj(routine);

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

  private Command leftTrenchMidAcross() {
    var routine = autoFactory.newRoutine("Left Trench Mid Across");
    var trajectory0 = ChoreoTraj.LeftTrenchMidAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchMidAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchMidAcross$2.asAutoTraj(routine);

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

  private Command leftTrenchFarAcross() {
    var routine = autoFactory.newRoutine("Left Trench Far Across");
    var trajectory0 = ChoreoTraj.LeftTrenchFarAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchFarAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchFarAcross$2.asAutoTraj(routine);

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

  private Command leftBumpMidAcross() {
    var routine = autoFactory.newRoutine("Left Bump Mid Across");
    var trajectory0 = ChoreoTraj.LeftBumpMidAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftBumpMidAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftBumpMidAcross$2.asAutoTraj(routine);

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

  private Command rightBumpMidAcross() {
    var routine = autoFactory.newRoutine("Right Bump Mid Across");
    var trajectory0 = ChoreoTraj.RightBumpMidAcross$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightBumpMidAcross$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightBumpMidAcross$2.asAutoTraj(routine);

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

  // Variants of the centerline autos that go back for seconds at the depot/outpost. We primarily
  // used the depot ones because outpost was inaccessible.
  // We aren't fast enough to go for a double dip in the neutral zone, so we do this instead.
  private Command leftTrenchFarAcrossOutpost() {
    var routine = autoFactory.newRoutine("Left Trench Far Across Outpost");
    var trajectory0 = ChoreoTraj.LeftTrenchFarAcrossOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchFarAcrossOutpost$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchFarAcrossOutpost$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftTrenchFarAcrossOutpost$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftTrenchFarAcrossOutpost$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.LeftTrenchFarAcrossOutpost$5.asAutoTraj(routine);

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
                    .withTimeout(8),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(8)));

    return routine.cmd();
  }

  private Command leftTrenchFarDepot() {
    var routine = autoFactory.newRoutine("Left Trench Far Depot");
    var trajectory0 = ChoreoTraj.LeftTrenchFarDepot$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchFarDepot$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchFarDepot$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftTrenchFarDepot$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftTrenchFarDepot$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.LeftTrenchFarDepot$5.asAutoTraj(routine);

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
                    .withTimeout(8),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(8)));

    return routine.cmd();
  }

  private Command leftTrenchMidAcrossOutpost() {
    var routine = autoFactory.newRoutine("Left Trench Mid Across Outpost");
    var trajectory0 = ChoreoTraj.LeftTrenchMidAcrossOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchMidAcrossOutpost$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchMidAcrossOutpost$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftTrenchMidAcrossOutpost$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftTrenchMidAcrossOutpost$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.LeftTrenchMidAcrossOutpost$5.asAutoTraj(routine);

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
                    .withTimeout(8),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                    aimCommand.get(),
                    shootCommand.get(),
                    waitSeconds(1).andThen(feedCommand.get()))));

    return routine.cmd();
  }

  private Command leftTrenchMidDepot() {
    var routine = autoFactory.newRoutine("Left Trench Mid Depot");
    var trajectory0 = ChoreoTraj.LeftTrenchMidDepot$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.LeftTrenchMidDepot$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.LeftTrenchMidDepot$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.LeftTrenchMidDepot$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.LeftTrenchMidDepot$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.LeftTrenchMidDepot$5.asAutoTraj(routine);

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
                    .withTimeout(8),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(8)));

    return routine.cmd();
  }

  private Command RightTrenchFarOutpost() {
    var routine = autoFactory.newRoutine("Right Trench Far Outpost");
    var trajectory0 = ChoreoTraj.RightTrenchFarOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchFarOutpost$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchFarOutpost$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightTrenchFarOutpost$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.RightTrenchFarOutpost$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.RightTrenchFarOutpost$5.asAutoTraj(routine);

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
                    .withTimeout(8),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(8)));

    return routine.cmd();
  }

  private Command RightTrenchMidAcrossDepot() {
    var routine = autoFactory.newRoutine("Right Trench Mid Across Depot");
    var trajectory0 = ChoreoTraj.RightTrenchMidAcrossDepot$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchMidAcrossDepot$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchMidAcrossDepot$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightTrenchMidAcrossDepot$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.RightTrenchMidAcrossDepot$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.RightTrenchMidAcrossDepot$5.asAutoTraj(routine);

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
                    .withTimeout(8),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(8)));

    return routine.cmd();
  }

  private Command RightTrenchMidOutpost() {
    var routine = autoFactory.newRoutine("Right Trench Mid Outpost");
    var trajectory0 = ChoreoTraj.RightTrenchMidOutpost$0.asAutoTraj(routine);
    var trajectory1 = ChoreoTraj.RightTrenchMidOutpost$1.asAutoTraj(routine);
    var trajectory2 = ChoreoTraj.RightTrenchMidOutpost$2.asAutoTraj(routine);
    var trajectory3 = ChoreoTraj.RightTrenchMidOutpost$3.asAutoTraj(routine);
    var trajectory4 = ChoreoTraj.RightTrenchMidOutpost$4.asAutoTraj(routine);
    var trajectory5 = ChoreoTraj.RightTrenchMidOutpost$5.asAutoTraj(routine);

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
                    .withTimeout(8),
                trajectory3.cmd(),
                trajectory4.cmd().deadlineFor(intakeCommand.get()),
                trajectory5.cmd(),
                parallel(
                        aimCommand.get(),
                        shootCommand.get(),
                        waitSeconds(1).andThen(feedCommand.get()))
                    .withTimeout(8)));

    return routine.cmd();
  }
}
