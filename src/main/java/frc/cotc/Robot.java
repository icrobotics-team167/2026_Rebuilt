// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.feeder.*;
import frc.cotc.intake.IntakePivot;
import frc.cotc.intake.IntakePivotIO;
import frc.cotc.intake.IntakePivotIOPhoenix;
import frc.cotc.intake.IntakeRoller;
import frc.cotc.intake.IntakeRollerIO;
import frc.cotc.intake.IntakeRollerIOPhoenix;
import frc.cotc.shooter.FlywheelIO;
import frc.cotc.shooter.FlywheelIOPhoenix;
import frc.cotc.shooter.FlywheelIOSim;
import frc.cotc.shooter.HoodIO;
import frc.cotc.shooter.HoodIOPhoenix;
import frc.cotc.shooter.HoodIOSim;
import frc.cotc.shooter.Shooter;
import frc.cotc.shooter.TurretIO;
import frc.cotc.shooter.TurretIOPhoenix;
import frc.cotc.shooter.TurretIOSim;
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

    Logger.start();

    CommandScheduler.getInstance().onCommandInitialize(CommandsLogging::commandStarted);
    CommandScheduler.getInstance().onCommandFinish(CommandsLogging::commandEnded);
    CommandScheduler.getInstance().onCommandInterrupt(CommandsLogging::logInterrupts);

    var swerve =
        new Swerve(
            switch (mode) {
              case REAL -> new SwerveIOReal();
              case SIM -> new SwerveIOSim();
              case REPLAY -> new SwerveIOReplay();
            },
            new AprilTagPoseEstimator("FrontLeft"),
            new AprilTagPoseEstimator("FrontRight"));
    var controller = new CommandXboxController(0);

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

    intakePivot.setDefaultCommand(intakePivot.extend());
    controller.rightTrigger().whileTrue(intakeRoller.intake());

    var beltFloor =
        new BeltFloor(
            switch (mode) {
              case REAL -> new BeltFloorIOPhoenix();
              case SIM, REPLAY -> new BeltFloorIO() {};
            });
    var raceway =
        new Raceway(
            switch (mode) {
              case REAL -> new RacewayIOPhoenix();
              case SIM, REPLAY -> new RacewayIO() {};
            });
    var turretFeeder =
        new TurretFeeder(
            switch (mode) {
              case REAL -> new TurretFeederIOPhoenix();
              case SIM, REPLAY -> new TurretFeederIO() {};
            });

    var delaySeconds = 0.5;
    controller
        .leftBumper()
        .onTrue(
            parallel(
                    turretFeeder
                        .runFeeder()
                        .withDeadline(
                            waitUntil(() -> !controller.leftBumper().getAsBoolean())
                                .withName("Wait until bumper release")
                                .andThen(waitSeconds(delaySeconds * 2).withName("Delay shutdown")))
                        .withName("Start up/Shut down feeder"),
                    waitSeconds(delaySeconds)
                        .withName("Delay startup")
                        .andThen(raceway.runRaceway())
                        .withDeadline(
                            waitUntil(() -> !controller.leftBumper().getAsBoolean())
                                .withName("Wait until bumper release")
                                .andThen(waitSeconds(delaySeconds).withName("Delay shutdown")))
                        .withName("Start up/Shut down raceway"),
                    waitSeconds(delaySeconds * 2)
                        .withName("Delay startup")
                        .andThen(beltFloor.runBelt())
                        .onlyWhile(controller.leftBumper())
                        .withName("Start up/Shut down belt"))
                .withName("Start up/Shut down feed system"));

    Supplier<Translation2d> translationalInputSupplier =
        () -> {
          var x = -controller.getLeftY();
          var y = -controller.getLeftX();
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
          var omega = -controller.getRightX();
          var deadbandedOmegaMag = MathUtil.applyDeadband(Math.abs(omega), 0.05);
          return omega * deadbandedOmegaMag;
        };

    swerve.setDefaultCommand(swerve.teleopDrive(translationalInputSupplier, omegaInputSupplier));
    controller
        .rightTrigger()
        .whileTrue(
            either(
                    swerve.aimAtTarget(translationalInputSupplier, Shooter.ShotTarget.RED_HUB),
                    swerve.aimAtTarget(translationalInputSupplier, Shooter.ShotTarget.BLUE_HUB),
                    Robot::isOnRed)
                .withName("Aim at target"));

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

    var shooter =
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
            },
            switch (mode) {
              case REAL -> new TurretIOPhoenix();
              case SIM -> new TurretIOSim();
              case REPLAY -> new TurretIO() {};
            },
            swerve::getPose,
            swerve::getFieldSpeeds);
    controller
        .leftTrigger()
        .whileTrue(
            either(
                    shooter.shootAt(Shooter.ShotTarget.RED_HUB),
                    shooter.shootAt(Shooter.ShotTarget.BLUE_HUB),
                    Robot::isOnRed)
                .withName("Shoot at alliance hub"));
    controller
        .leftBumper()
        .whileTrue(
            parallel(shooter.pass(), swerve.pass(translationalInputSupplier)).withName("Pass"));

    autos = new Autos(swerve);

    RobotModeTriggers.autonomous()
        .whileTrue(deferredProxy(autos::getSelectedCommand).withName("Auto Command"))
        .onFalse(runOnce(autos::clear));
  }

  @Override
  public void disabledPeriodic() {
    autos.update();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 39);

    canivoreSignals.refreshAll();
    rioSignals.refreshAll();
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

  public static boolean isOnRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }

  public static Supplier<Pose2d> groundTruthPoseSupplier;
}
