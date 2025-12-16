// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Arrays;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization method. Do not perform any initialization here.
   *
   * <p>If you change your main Robot class (name), change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(() -> new Robot(Arrays.asList(args).contains("--replay")));
  }
}
