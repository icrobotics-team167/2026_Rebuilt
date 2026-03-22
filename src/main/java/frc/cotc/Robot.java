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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.autos.Autos;
import frc.cotc.feeder.*;
import frc.cotc.intake.IntakePivot;
import frc.cotc.intake.IntakePivotIO;
import frc.cotc.intake.IntakePivotIOPhoenix;
import frc.cotc.intake.IntakeRoller;
import frc.cotc.intake.IntakeRollerIO;
import frc.cotc.intake.IntakeRollerIOPhoenix;
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

    RobotController.setBrownoutVoltage(6);

    Logger.start();

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
    var primary = new CommandXboxController(0);

    var intakeRoller =
        new IntakeRoller(
            switch (mode) {
              case REAL -> new IntakeRollerIOPhoenix();
              case SIM, REPLAY -> new IntakeRollerIO() {};
            });

    var intakePivot =
        new IntakePivot(
            switch (mode) {
              case REAL -> new IntakePivotIOPhoenix();
              case SIM, REPLAY -> new IntakePivotIO() {};
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
                parallel(
                    beltFloor.runBelt(),
                    raceway.runRaceway(),
                    intakePivot.agitate().asProxy(),
                    intakeRoller.intake().asProxy()),
            intakeRoller);
    CommandScheduler.getInstance().schedule(autos.warmup());

    RobotModeTriggers.autonomous()
        .whileTrue(deferredProxy(autos::getSelectedCommand).withName("Auto Command"))
        .onFalse(runOnce(autos::clear));
    RobotModeTriggers.teleop().onTrue(runOnce(Shifts::initialize));

    turretFeeder.setDefaultCommand(turretFeeder.runFeeder());
    primary
        .rightTrigger()
        .and(
            () ->
                switch (shotTarget) {
                  case RED_HUB, BLUE_HUB -> isOkayToShoot;
                  default -> true;
                })
        .whileTrue(parallel(beltFloor.runBelt(), raceway.runRaceway()).withName("Feed"));

    Supplier<Translation2d> translationalInputSupplier =
        () -> {
          var x = -primary.getLeftY();
          var y = -primary.getLeftX();
          var magnitude = Math.hypot(x, y);
          if (magnitude > 1e-6) {
            var normX = x / magnitude;
            var normY = y / magnitude;
            var deadbandedMagnitude = MathUtil.applyDeadband(Math.min(magnitude, 1), 0.05);
            var squaredDeadbandedMagnitude = deadbandedMagnitude * deadbandedMagnitude;
            return new Translation2d(
                normX * squaredDeadbandedMagnitude, normY * squaredDeadbandedMagnitude);
          } else {
            return Translation2d.kZero;
          }
        };

    DoubleSupplier omegaInputSupplier =
        () -> {
          var omega = -primary.getRightX();
          var deadbandedOmegaMag = MathUtil.applyDeadband(Math.abs(omega), 0.05);
          return omega * deadbandedOmegaMag * deadbandedOmegaMag;
        };

    swerve.setDefaultCommand(swerve.teleopDrive(translationalInputSupplier, omegaInputSupplier));
    primary.rightBumper().whileTrue(swerve.slowTeleopDrive());
    primary
        .povLeft()
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
                    var deadbandedMagnitude = MathUtil.applyDeadband(Math.min(magnitude, 1), 0.05);
                    var squaredDeadbandedMagnitude = deadbandedMagnitude * deadbandedMagnitude;
                    return new Translation2d(
                        normX * squaredDeadbandedMagnitude, normY * squaredDeadbandedMagnitude);
                  } else {
                    return Translation2d.kZero;
                  }
                }));
    primary.povRight().whileTrue(swerve.brake());

    primary
        .b()
        .whileTrue(
            parallel(shooter.idle(), beltFloor.idle(), raceway.idle(), turretFeeder.idle())
                .ignoringDisable(true)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .withName("Disable shooting"));

    shooter.setDefaultCommand(shooter.idleRun());
    primary
        .leftBumper()
        .whileTrue(
            parallel(swerve.aimAtTarget(translationalInputSupplier), shooter.sotm())
                .withName("Shoot"));

    intakePivot.setDefaultCommand(
        parallel(intakePivot.extend(), intakeRoller.intake().withTimeout(0.25).asProxy())
            .withName("Extend"));
    primary
        .a()
        .toggleOnTrue(
            parallel(intakePivot.retract(), intakeRoller.intake().withTimeout(0.25).asProxy())
                .withName("Retract"));
    primary
        .x()
        .whileTrue(parallel(intakePivot.agitate(), intakeRoller.intake()).withName("Agitate"));
    primary
        .leftTrigger()
        .whileTrue(parallel(intakePivot.extend(), intakeRoller.intake()).withName("Intake"));
    primary.y().whileTrue(intakeRoller.outtake());

    primary
        .back()
        .and(primary.start())
        .debounce(2)
        .toggleOnTrue(
            parallel(
                    shooter.idle(),
                    beltFloor.idle(),
                    raceway.idle(),
                    turretFeeder.idle(),
                    swerve.fastTeleopDrive())
                .withName("Boost"));

    new Trigger(() -> shiftInfo != null && shiftInfo.remainingTime() < 5)
        .onTrue(
            run(() -> primary.setRumble(GenericHID.RumbleType.kBothRumble, 1))
                .withTimeout(0.25)
                .finallyDo(() -> primary.setRumble(GenericHID.RumbleType.kBothRumble, 0)));
    new Trigger(() -> shiftInfo != null && shiftInfo.active())
        .onChange(
            run(() -> primary.setRumble(GenericHID.RumbleType.kBothRumble, 1))
                .withTimeout(0.5)
                .finallyDo(() -> primary.setRumble(GenericHID.RumbleType.kBothRumble, 0)));
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
    Threads.setCurrentThreadPriority(true, 1);

    canivoreSignals.refreshAll();
    rioSignals.refreshAll();
    updateTarget();
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
    shiftInfo = Shifts.getOfficialShiftInfo();
    Logger.recordOutput("ShiftInfo/CurrentShift", shiftInfo.currentShift());
    Logger.recordOutput("ShiftInfo/Active", shiftInfo.active());
    Logger.recordOutput("ShiftInfo/ElapsedTime", shiftInfo.elapsedTime());
    Logger.recordOutput("ShiftInfo/RemainingTime", shiftInfo.remainingTime());
    var timeOfFlight =
        (shotTarget == SOTM.ShotTarget.BLUE_HUB || shotTarget == SOTM.ShotTarget.RED_HUB)
            ? result.timeOfFlightSeconds()
            : 0.7;
    var adjustedShiftInfo = Shifts.getAdjustedShiftInfo(timeOfFlight);
    Logger.recordOutput("ShiftInfo/TimeLeftTillShooting", adjustedShiftInfo.remainingTime());
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
      Logger.recordOutput("Swerve/Ground Truth Pose", groundTruthPoseSupplier.get());
    }
    SmartDashboard.putData(CommandScheduler.getInstance());

    Threads.setCurrentThreadPriority(false, 10);
  }

  private void updateTarget() {
    var currentPose = swerve.getPose();
    if (Robot.isOnRed()) {
      if (currentPose.getX() > FieldConstants.Hub.oppTopCenterPoint.getX()) {
        shotTarget = SOTM.ShotTarget.RED_HUB;
      } else if (currentPose.getY() > FieldConstants.fieldWidth / 2) {
        shotTarget = SOTM.ShotTarget.RED_TOP_GROUND;
      } else {
        shotTarget = SOTM.ShotTarget.RED_BOTTOM_GROUND;
      }
    } else {
      if (currentPose.getX() < FieldConstants.Hub.topCenterPoint.getX()) {
        shotTarget = SOTM.ShotTarget.BLUE_HUB;
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
