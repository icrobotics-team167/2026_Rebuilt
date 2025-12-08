// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;

  public Swerve(SwerveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {}

  public Command drive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return run(() -> {});
  }
}
