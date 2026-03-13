// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final HoodIO hoodIO;
  private final FlywheelIO flywheelIO;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final InterpolatingDoubleTreeMap flywheelSpeedToProjectileSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap projectileSpeedToFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  public Shooter(HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.hoodIO = hoodIO;
    this.flywheelIO = flywheelIO;

    addMapping(0, 0);

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
    Logger.recordOutput("Shooter/Actual projectile speed meters per sec", (flywheelInputs.velRotPerSec));
    Logger.recordOutput("Shooter/Target projectile speed meters per sec", targetSpeedMetersPerSec);
  }

  private final double baseTargetSpeedMetersPerSec = 5;
  private double targetSpeedMetersPerSec = baseTargetSpeedMetersPerSec;

  public Command idleRun() {
    return run(() -> {
      targetSpeedMetersPerSec = baseTargetSpeedMetersPerSec;
      flywheelIO.runVel(targetSpeedMetersPerSec);
    });
  }

  private SOTM.SOTMResult sotmResult;

  public void setSOTMResult(SOTM.SOTMResult result) {
    this.sotmResult = result;
  }

  public Command sotm() {
    return run(() -> {
      if (sotmResult == null) return;
      hoodIO.runPitch(sotmResult.pitchRad());
      flywheelIO.runVel(projectileSpeedToFlywheelSpeedMap.get(sotmResult.shotSpeedMetersPerSecond()));
    });
  }
}
