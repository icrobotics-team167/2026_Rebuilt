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
  private final HoodIO hoodIO;
  private final FlywheelIO flywheelIO;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final InterpolatingDoubleTreeMap flywheelSpeedToProjectileSpeedMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap projectileSpeedToFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  public Shooter(HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;

    addMapping(0, 0);
    addMapping(37.8, 6.627);
    addMapping(38.2, 7.03);
    addMapping(40.8, 7.67);
    addMapping(42, 8.019);
    addMapping(43.8, 8.377);
  }

  private void addMapping(double flywheelSpeedRotPerSec, double projectileSpeedMetersPerSec) {
    flywheelSpeedToProjectileSpeedMap.put(flywheelSpeedRotPerSec, projectileSpeedMetersPerSec);
    projectileSpeedToFlywheelSpeedMap.put(projectileSpeedMetersPerSec, flywheelSpeedRotPerSec);
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    Logger.recordOutput("Shooter/Target flywheel speed rot per sec", targetSpeedRotPerSec);
  }

  private final double baseTargetSpeedRotPerSec = 36;
  private double targetSpeedRotPerSec = baseTargetSpeedRotPerSec;

  private final double presetAngle = Units.degreesToRadians(60); // placeholder

  public Command idleRun() {
    return run(
        () -> {
          targetSpeedRotPerSec = baseTargetSpeedRotPerSec;
          flywheelIO.runVel(baseTargetSpeedRotPerSec);
          hoodIO.runPitch(presetAngle);
        });
  }

  @Override
  public Command idle() {
    return run(
        () -> {
          targetSpeedRotPerSec = 0;
          flywheelIO.stop();
        });
  }

  private SOTM.SOTMResult sotmResult;

  public void setSOTMResult(SOTM.SOTMResult result) {
    this.sotmResult = result;
  }

  public Command sotm() {
    return run(
        () -> {
          if (sotmResult == null) return;
          hoodIO.runPitch(sotmResult.pitchRad());
          targetSpeedRotPerSec =
              projectileSpeedToFlywheelSpeedMap.get(sotmResult.shotSpeedMetersPerSecond());
          flywheelIO.runVel(targetSpeedRotPerSec);
        });
  }

  public Command raiseHood() {
    return run(() -> hoodIO.runPitch(Units.degreesToRadians(60)));
  }

  public Command lowerHood() {
    return run(() -> hoodIO.runPitch(Units.degreesToRadians(40)));
  }
}
