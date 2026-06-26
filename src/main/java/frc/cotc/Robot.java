// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.autos.Autos;
import frc.cotc.feeder.*;
import frc.cotc.intake.*;
import frc.cotc.shooter.*;
import frc.cotc.swerve.*;
import frc.cotc.vision.AprilTagPoseEstimator;
import java.io.FileNotFoundException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  private final Autos autos;
  public static Mode mode;

  // Batch calling .refreshAll() on CTRE CAN signals using StatusSignalCollection is much faster
  // than calling .refresh() on each signal individually, since the batch call doesn't have to
  // hop the overhead-intensive JNI barrier as often.
  public static final StatusSignalCollection canivoreSignals = new StatusSignalCollection();
  public static final StatusSignalCollection rioSignals = new StatusSignalCollection();

  public static final CANBus rioBus = CANBus.roboRIO();

  public static SOTM.ShotTarget shotTarget = SOTM.ShotTarget.BLUE_HUB;

  private final Swerve swerve;
  private final Shooter shooter;

  private boolean isOkayToShoot = true;

  private Shifts.ShiftInfo shiftInfo;

  @SuppressWarnings({"UnreachableCode", "ConstantValue"})
  public Robot(boolean isReplay) {
    // If this is erroring, hit build
    // Compiling auto-generates the BuildConstants file
    Logger.recordMetadata("Project", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("Git branch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Git commit date", BuildConstants.GIT_DATE);
    Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    //noinspection ConstantValue
    Logger.recordMetadata("Uncommited changes", BuildConstants.DIRTY == 1 ? "True" : "False");
    Logger.recordMetadata("Compile date", BuildConstants.BUILD_DATE);

    // Always set it to REAL if running on real hardware.
    // Failsafe for if the replay flag somehow got passed onto the RoboRIO deploy.
    mode = Robot.isReal() ? Mode.REAL : (isReplay ? Mode.REPLAY : Mode.SIM);

    switch (mode) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        LoggedPowerDistribution.getInstance(); // Enables power distribution logging

        var serialNumber = RobotController.getSerialNumber();
        Logger.recordMetadata("RoboRIO Serial number", serialNumber);

        SignalLogger.start(); // Start logging Phoenix CAN signals
      }
      case SIM -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to the project's logs folder
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        SignalLogger.start(); // Start logging Phoenix CAN signals
      }
      case REPLAY -> {
        setUseTiming(false); // Run as fast as possible
        String logPath;
        try {
          logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope
        } catch (Exception e) {
          throw new RuntimeException(
              new FileNotFoundException(
                  "Failed to ask the user for a log file! If you are using IntelliJ, please open the "
                      + "log file in AdvantageScope and try again!"));
        }
        // Note: User prompting will fail and crash on IntelliJ, so have the log open in
        // AScope.
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_replay"))); // Save outputs to a new log
      }
    }

    // Lower the brownout voltage to gain more headroom for aggressive current draw
    // xd
    RobotController.setBrownoutVoltage(6);

    Logger.start();

    // Register the commands logger to the scheduler
    CommandScheduler.getInstance().onCommandInitialize(CommandsLogging::commandStarted);
    CommandScheduler.getInstance().onCommandFinish(CommandsLogging::commandEnded);
    CommandScheduler.getInstance().onCommandInterrupt(CommandsLogging::logInterrupts);

    swerve =
        new Swerve(
            switch (mode) {
              case REAL -> new SwerveIOReal();
              case SIM -> new SwerveIOSim();
              case REPLAY -> new SwerveIOReplay();
            },
            new AprilTagPoseEstimator("Front"),
            new AprilTagPoseEstimator("Left"),
            new AprilTagPoseEstimator("Back"),
            new AprilTagPoseEstimator("Right"));
    var primary = new CommandXboxControllerWithRumble(0);

    var intake =
        new Intake(
            switch (mode) {
              case REAL -> new IntakePivotIOPhoenix();
              case SIM, REPLAY -> new IntakePivotIO() {};
            },
            switch (mode) {
              case REAL -> new IntakeRollerIOPhoenix();
              case SIM, REPLAY -> new IntakeRollerIO() {};
            });

    var beltFloor =
        new BeltFloor(
            switch (mode) {
              case REAL -> new BeltFloorIOPhoenix();
              case SIM, REPLAY -> new BeltFloorIO() {};
            });
    var turretFeeder =
        new TurretFeeder(
            switch (mode) {
              case REAL -> new TurretFeederIOPhoenix();
              case SIM, REPLAY -> new TurretFeederIO() {};
            });
    var raceway =
        new Raceway(
            switch (mode) {
              case REAL -> new RacewayIOPhoenix();
              case SIM, REPLAY -> new RacewayIO() {};
            });

    new Trigger(
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
        .onTrue(swerve.setToBlue());
    new Trigger(
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
        .onTrue(swerve.setToRed());

    shooter =
        new Shooter(
            switch (mode) {
              case REAL -> new HoodIOPhoenix();
              case SIM -> new HoodIOSim();
              case REPLAY -> new HoodIO() {};
            },
            switch (mode) {
              case REAL -> new FlywheelIOPhoenix();
              case SIM -> new FlywheelIOSim();
              case REPLAY -> new FlywheelIO() {};
            });

    autos =
        new Autos(
            swerve,
            shooter,
            () ->
                parallel(beltFloor.runBelt(), raceway.runRaceway(), intake.agitate().asProxy())
                    .withName("Feed"),
            () -> intake.intake().asProxy());
    CommandScheduler.getInstance().schedule(autos.warmup());

    RobotModeTriggers.autonomous()
        // deferredProxy will call the method to grab the command every time it's called, allowing
        // for the command that the method returns to change when the dashboard selection changes,
        // instead of being fixed from boot.
        .whileTrue(deferredProxy(autos::getSelectedCommand).withName("Auto Command"))
        .onFalse(runOnce(autos::clear));
    // On teleop enable, start the timer for shift info
    RobotModeTriggers.teleop().onTrue(runOnce(Shifts::initialize));

    turretFeeder.setDefaultCommand(turretFeeder.runFeeder());
    primary
        .rightTrigger()
        .and(DriverStation::isEnabled)
        .and(
            () ->
                switch (shotTarget) {
                  // If shooting at a hub, use the shift timings or the override to determine if
                  // it's okay to shoot
                  case RED_HUB, BLUE_HUB -> isOkayToShoot || primary.getHID().getPOV() == 0;
                  // If passing, shooting anytime is okay
                  default -> true;
                })
        .whileTrue(parallel(beltFloor.runBelt(), raceway.runRaceway()).withName("Feed"));

    Supplier<Translation2d> translationalInputSupplier =
        () -> {
          // Applying the deadband to the x and y axes individually feels bad and isn't how the
          // stick is expected to operate given the circular nature of the stick's movement area,
          // so we apply the deadband to the magnitude of the control input vector, then rescale
          // the x and y axes to match.
          var x = -primary.getLeftY();
          var y = -primary.getLeftX();
          var magnitude = Math.hypot(x, y);
          if (magnitude > 1e-6) {
            var normX = x / magnitude;
            var normY = y / magnitude;
            var deadbandedMagnitude = MathUtil.applyDeadband(Math.min(magnitude, 1), 0.075);
            var squaredDeadbandedMagnitude = Math.pow(deadbandedMagnitude, 1.5);
            return new Translation2d(
                // Square the inputs to allow for more precision at slow speeds
                // NOTE: Highly dependent on driver preference. Edmund complained about the robot
                // feeling jumpy due to a sharp transition between the slow speed regime and the
                // high speed regime, so maybe don't do this.
                normX * squaredDeadbandedMagnitude, normY * squaredDeadbandedMagnitude);
          } else {
            return Translation2d.kZero;
          }
        };

    DoubleSupplier omegaInputSupplier =
        () -> {
          var omega = -primary.getRightX();
          var deadbandedOmegaMag = MathUtil.applyDeadband(Math.abs(omega), 0.075);
          // Cube it instead of square it to make the slowdown at low inputs even greater
          // Edmund requested that the spinning be more precise at slow speeds, so I did this.
          return omega * deadbandedOmegaMag * deadbandedOmegaMag;
        };

    swerve.setDefaultCommand(swerve.teleopDrive(translationalInputSupplier, omegaInputSupplier));
    // Slow down when needed
    primary.rightBumper().and(DriverStation::isEnabled).whileTrue(swerve.slowTeleopDrive());
    // Mode to point the right stick to aim the direction instead of using the right stick to
    // move it manually.
    // Edmund requested this to automate aiming, but the imprecision of aiming with a stick
    // direction and bad sightlines in some parts of the field made this not worth.
    primary
        .povLeft()
        .and(DriverStation::isEnabled)
        .whileTrue(
            swerve.faceAngle(
                translationalInputSupplier,
                () -> {
                  var x = -primary.getRightY();
                  var y = -primary.getRightX();
                  var magnitude = Math.hypot(x, y);
                  if (magnitude > 1e-6) {
                    var normX = x / magnitude;
                    var normY = y / magnitude;
                    var deadbandedMagnitude = MathUtil.applyDeadband(Math.min(magnitude, 1), 0.25);
                    var squaredDeadbandedMagnitude = deadbandedMagnitude * deadbandedMagnitude;
                    return new Translation2d(
                        normX * squaredDeadbandedMagnitude, normY * squaredDeadbandedMagnitude);
                  } else {
                    return Translation2d.kZero;
                  }
                }));
    // Brake the wheels in an X to lock the robot in place. Good for defense. (And for preventing
    // the robot from sliding around in the trunk of a car.)
    primary.povRight().and(DriverStation::isEnabled).whileTrue(swerve.brake());

    // Hold b to speed up the drivetrain top speed and disable every other subsystem to maximize
    // available power for acceleration.
    primary
        .b()
        .and(DriverStation::isEnabled)
        .whileTrue(
            parallel(
                    shooter.idle(),
                    beltFloor.idle(),
                    raceway.idle(),
                    turretFeeder.idle(),
                    swerve.fastTeleopDrive())
                .ignoringDisable(true)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .withName("Disable shooting")); // Should've been named "Boost"

    shooter.setDefaultCommand(shooter.idleRun());
    // Aim at target in yaw (swerve) and pitch/speed (shooter)
    primary
        .leftBumper()
        .and(DriverStation::isEnabled)
        .whileTrue(swerve.aimAtTarget(translationalInputSupplier))
        .whileTrue(shooter.sotm());

    intake.setDefaultCommand(intake.fastExtend());
    primary.a().and(DriverStation::isEnabled).toggleOnTrue(intake.retract());
    primary.x().and(DriverStation::isEnabled).whileTrue(intake.agitate());
    primary.leftTrigger().and(DriverStation::isEnabled).whileTrue(intake.intake());
    primary
        .y()
        .and(DriverStation::isEnabled)
        .whileTrue(parallel(intake.outtake(), beltFloor.runBackwards()));

    // Hold back and start for 2 seconds to emergency shut off the shooter
    // In the case of a systems failure and we need to transition to defense, this does the job
    // of holding b (see above) but without actually holding b.
    primary
        .back()
        .and(primary.start())
        .and(DriverStation::isEnabled)
        .debounce(2)
        .toggleOnTrue(
            parallel(shooter.idle(), beltFloor.idle(), raceway.idle(), turretFeeder.idle())
                .withName("Boost")) // Should've been named "Disable shooting"
        .onTrue(
            // Rumble the controller 3 times to indicate a successful activation/deaactivation of
            // the shooter emergency shutoff
            sequence(
                primary.rumble(0.2),
                waitSeconds(0.1),
                primary.rumble(0.2),
                waitSeconds(0.1),
                primary.rumble(0.2)));

    // Rumble the controller on shift changes (5 seconds and 0 seconds left)
    new Trigger(() -> shiftInfo != null && shiftInfo.remainingTime() < 5)
        .onTrue(primary.rumble(0.25));
    new Trigger(() -> shiftInfo != null && shiftInfo.active()).onChange(primary.rumble(0.5));
  }

  @Override
  public void disabledPeriodic() {
    autos.update();
  }

  // The shooter will lag behind the target position, so try to look a little further into the
  // future to compensate
  // TODO: Tune
  @SuppressWarnings("FieldCanBeLocal")
  private final double LOOK_AHEAD_SECONDS = 0.2;

  @Override
  public void robotPeriodic() {
    // Use the RoboRIO's Linux-RT kernel to prioritize our code over other processes. This speeds
    // up loop times and reduces variability, at the cost of exploding necessary processes if we
    // take too long.
    // Some example code out there uses 99 priority, THIS IS A BAD IDEA.
    // 99 priority means NOTHING ELSE CAN RUN while our code is running, which makes the chance of
    // an exploding process much higher. 1 priority is enough to get the benefits of Linux-RT
    // without the risk of exploding processes.
    // Learned that the hard way.
    Threads.setCurrentThreadPriority(true, 1);

    // In replay, we don't need to refresh the signals since there's no signals to refresh.
    if (mode != Mode.REPLAY) {
      canivoreSignals.refreshAll();
      rioSignals.refreshAll();
    }
    updateTarget();
    // Calculate the SOTM result to make a shot into the desired target.
    var fieldChassisSpeeds = swerve.getFieldSpeeds();
    var result =
        SOTM.calculate(
            swerve
                .getPose()
                .plus(
                    new Transform2d(
                        fieldChassisSpeeds.vxMetersPerSecond * LOOK_AHEAD_SECONDS,
                        fieldChassisSpeeds.vyMetersPerSecond * LOOK_AHEAD_SECONDS,
                        new Rotation2d(
                            fieldChassisSpeeds.omegaRadiansPerSecond * LOOK_AHEAD_SECONDS))),
            fieldChassisSpeeds,
            shotTarget);
    Logger.recordOutput("Shooter/Target", new Pose2d(shotTarget.targetLocation, Rotation2d.kZero));
    swerve.setSOTMResult(result);
    shooter.setSOTMResult(result);
    // Calculate the current shift info
    shiftInfo = Shifts.getOfficialShiftInfo();
    Logger.recordOutput("ShiftInfo/CurrentShift", shiftInfo.currentShift());
    Logger.recordOutput("ShiftInfo/Active", shiftInfo.active());
    Logger.recordOutput("ShiftInfo/ElapsedTime", (int) Math.ceil(shiftInfo.elapsedTime()));
    Logger.recordOutput("ShiftInfo/RemainingTime", (int) Math.ceil(shiftInfo.remainingTime()));
    // Using the time of flight of the current possible shot, shift the timings of the shifts to
    // account for the fact that the shot will be fired now but land in the future.
    var timeOfFlight =
        (shotTarget == SOTM.ShotTarget.BLUE_HUB || shotTarget == SOTM.ShotTarget.RED_HUB)
            ? result.timeOfFlightSeconds()
            : 0.7; // Rough estimate of the ToF at the bump, since ToF from neutral zone to hub
    // isn't useful
    var adjustedShiftInfo = Shifts.getAdjustedShiftInfo(timeOfFlight);
    Logger.recordOutput(
        "ShiftInfo/TimeLeftTillShooting", (int) Math.ceil(adjustedShiftInfo.remainingTime()));
    Logger.recordOutput(
        "ShiftInfo/OkayToShoot",
        adjustedShiftInfo.active()
            && (shotTarget == SOTM.ShotTarget.BLUE_HUB || shotTarget == SOTM.ShotTarget.RED_HUB));
    isOkayToShoot = adjustedShiftInfo.active();
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    CommandsLogging.logRunningCommands();
    CommandsLogging.logRequiredSubsystems();
    Logger.recordOutput(
        "LoggedRobot/MemoryUsageMB",
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6);
    Logger.recordOutput("IsOnRed", isOnRed());
    if (groundTruthPoseSupplier != null) {
      // Record the ground truth pose from the simulation to compare against the odometry pose
      Logger.recordOutput("Swerve/Ground Truth Pose", groundTruthPoseSupplier.get());
    }
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Go back to normal thread priority to allow other processes to run.
    Threads.setCurrentThreadPriority(false, 1);
  }

  /**
   * Automatically select a target based on the current pose.
   *
   * <p>If we're in our alliance zone, aim at our hub. If we're in the neutral zone, aim at
   * whichever corner of the alliance zone is closer to us from our position.
   */
  private void updateTarget() {
    var currentPose = swerve.getPose();
    if (Robot.isOnRed()) {
      // If we're on red and in our alliance zone, target red
      if (currentPose.getX() > FieldConstants.Hub.oppTopCenterPoint.getX()) {
        shotTarget = SOTM.ShotTarget.RED_HUB;
        // If we're on the top side of the field, (left from DS perspective) aim at the passing
        // location on the top side of the field. Otherwise, aim at the bottom side of the field.
      } else if (currentPose.getY() > FieldConstants.fieldWidth / 2) {
        shotTarget = SOTM.ShotTarget.RED_TOP_GROUND;
      } else {
        shotTarget = SOTM.ShotTarget.RED_BOTTOM_GROUND;
      }
    } else {
      // If we're on blue and in our alliance zone, target blue
      if (currentPose.getX() < FieldConstants.Hub.topCenterPoint.getX()) {
        shotTarget = SOTM.ShotTarget.BLUE_HUB;
        // If we're on the top side of the field, (right from DS perspective) aim at the passing
        // location on the top side of the field. Otherwise, aim at the bottom side of the field.
      } else if (currentPose.getY() > FieldConstants.fieldWidth / 2) {
        shotTarget = SOTM.ShotTarget.BLUE_TOP_GROUND;
      } else {
        shotTarget = SOTM.ShotTarget.BLUE_BOTTOM_GROUND;
      }
    }
  }

  public static boolean isOnRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }

  public static Supplier<Pose2d> groundTruthPoseSupplier;
}
