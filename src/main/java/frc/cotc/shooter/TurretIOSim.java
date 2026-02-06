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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.cotc.Robot;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), .25, 30),
          DCMotor.getKrakenX44(1).withReduction(30));

  private final PIDController pid = new PIDController(1, 0, 1); // Placeholder values

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sim.update(Robot.defaultPeriodSecs);

    inputs.thetaRad = sim.getAngularPositionRad();
    inputs.omegaRadPerSec = sim.getAngularVelocityRadPerSec();
  }

  @Override
  public void stop() {
    sim.setInputVoltage(0);
  }

  @Override
  public void runYaw(double thetaRad, double omegaRadPerSec) {
    sim.setInputVoltage(
        pid.calculate(sim.getAngularPositionRad(), thetaRad)
            + omegaRadPerSec / sim.getGearbox().KvRadPerSecPerVolt);
  }
}
