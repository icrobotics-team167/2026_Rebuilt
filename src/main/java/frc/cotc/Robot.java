// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.intake.Intake;
import frc.cotc.intake.IntakeIO;
import frc.cotc.intake.IntakeIOPhoenix;
import frc.cotc.shooter.FlywheelIO;
import frc.cotc.shooter.HoodIO;
import frc.cotc.shooter.Shooter;
import frc.cotc.shooter.TurretIO;
import frc.cotc.swerve.*;
import frc.cotc.vision.AprilTagPoseEstimator;
import java.io.FileNotFoundException;
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
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (interrupted, interrupting) -> {
              interrupting.ifPresent(
                  interrupter -> CommandsLogging.runningInterrupters.put(interrupter, interrupted));
              CommandsLogging.commandEnded(interrupted);
            });

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

    var intake =
        new Intake(
            switch (mode) {
              case REAL -> new IntakeIOPhoenix();
              case SIM, REPLAY -> new IntakeIO() {};
            });

    swerve.setDefaultCommand(
        swerve.teleopDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

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
            new HoodIO() {},
            new FlywheelIO() {},
            new TurretIO() {},
            swerve::getPose,
            swerve::getFieldSpeeds);
    RobotModeTriggers.teleop().whileTrue(shooter.shootAtHub());
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
