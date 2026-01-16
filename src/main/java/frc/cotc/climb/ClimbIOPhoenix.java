// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class ClimbIOPhoenix implements ClimbIO {
  private final TalonFX motor;

  public ClimbIOPhoenix() {
    // 0 as CAN placeholder
    motor = new TalonFX(0);

    var motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.posRad = Units.rotationsToRadians(0);
  }
}
