// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {

  public Robot() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
