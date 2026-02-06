// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.cotc.Robot;

public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(2), .25, 0.5),
          DCMotor.getKrakenX44(2).withReduction(0.5));

  private final PIDController pid = new PIDController(1, 0, 1); // Placeholders

  @Override
  public void stop() {
    sim.setInputVoltage(0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sim.update(Robot.defaultPeriodSecs);

    inputs.projectileVelMetersPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = sim.getInputVoltage();
  }

  @Override
  public void runVel(double projectileVelMetersPerSec) {
    sim.setInputVoltage(
        pid.calculate(sim.getAngularVelocityRadPerSec(), projectileVelMetersPerSec));
  }
}
