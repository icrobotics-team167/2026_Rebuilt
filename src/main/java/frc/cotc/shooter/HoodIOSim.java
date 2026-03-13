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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.cotc.Robot;

public class HoodIOSim implements HoodIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX44(1), .25, 30),
          DCMotor.getKrakenX44(1).withReduction(154.0 / 5),
          0,
          0,
          0,
          0,
          false,
          0); // placeholders

  private final PIDController pid = new PIDController(1, 0, 1);
  private final double kG = 1.0; // placeholder

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    sim.update(Robot.defaultPeriodSecs);

    inputs.thetaRad = sim.getAngleRads();
  }

  @Override
  public void runPitch(double thetaRad) {
    sim.setInputVoltage(
        pid.calculate(sim.getAngleRads(), thetaRad) + kG * Math.cos(sim.getAngleRads()));
  }
}
