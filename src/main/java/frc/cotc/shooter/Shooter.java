// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // Unlike the Intake where the individual components were coordinated together but moved
  // independently, everything in the shooter is moved together, so we put them in the same
  // subsystem.
  private final HoodIO hoodIO;
  private final FlywheelIO flywheelIO;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final InterpolatingDoubleTreeMap projectileSpeedToFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  public Shooter(HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;

    // This was measured by going different distances and seeing at what flywheel speed did the
    // fuel make it into the goal given the hood angle provided by the trajectory solver.
    projectileSpeedToFlywheelSpeedMap.put(0.0, 0.0);
    projectileSpeedToFlywheelSpeedMap.put(6.8, 68.0);
    projectileSpeedToFlywheelSpeedMap.put(7.4, 74.5);
    projectileSpeedToFlywheelSpeedMap.put(14.6, 153.5);
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    Logger.recordOutput("Shooter/Target flywheel speed rot per sec", targetSpeedRotPerSec);
    Logger.recordOutput("Shooter/Target hood pitch rad", targetPitchRad);
  }

  private final double baseTargetSpeedRotPerSec = 80;
  private double targetSpeedRotPerSec = baseTargetSpeedRotPerSec;

  private final double minAngle = Units.degreesToRadians(55);
  private double targetPitchRad = minAngle;

  /** Idles the flywheel at a base speed to conserve some momentum while consuming minimal power. */
  public Command idleRun() {
    return run(
        () -> {
          targetSpeedRotPerSec = baseTargetSpeedRotPerSec;
          flywheelIO.runVel(baseTargetSpeedRotPerSec);
          hoodIO.runPitch(minAngle);
          targetPitchRad = minAngle;
        });
  }

  /** Completely shuts off the flywheel for maximum power savings. */
  @Override
  public Command idle() {
    return run(
        () -> {
          targetSpeedRotPerSec = 0;
          flywheelIO.stop();
          hoodIO.runPitch(minAngle);
          targetPitchRad = minAngle;
        });
  }

  private SOTM.SOTMResult sotmResult;

  // HACK: The SOTM result needs to be shared between the otherwise entirely separated swerve and
  // shooter subsystems, requiring some method of transferring data. This is it.
  public void setSOTMResult(SOTM.SOTMResult result) {
    this.sotmResult = result;
  }

  /** Use the calculated SOTM result to run the flywheel and hood */
  public Command sotm() {
    return run(() -> {
          if (sotmResult == null) return;
          hoodIO.runPitch(sotmResult.pitchRad());
          targetPitchRad = sotmResult.pitchRad();
          targetSpeedRotPerSec =
              projectileSpeedToFlywheelSpeedMap.get(sotmResult.shotSpeedMetersPerSecond());
          flywheelIO.runVel(targetSpeedRotPerSec);
        })
        .withName("SOTM");
  }
}
