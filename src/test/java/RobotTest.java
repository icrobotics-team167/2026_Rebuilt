// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class RobotTest {
  @Test
  void initRobot() {
    Robot robot = new Robot(false);
    assertEquals(Robot.Mode.SIM, robot.mode);
  }
}
