// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

public class Constants {
  private Constants() {}

  public static final String MOTOR_DISCONNECT_ALERT_GROUP = "Motor Disconnects";

  public static final double frameWidth = 32.0;
  public static final double frameLength = 22.0;
  public static final double offset = 2.625;
  
  public static final double trackWidth = kFrontLeftYPos - kBackLeftYPos;
  public static final double trackLength =  kFrontLeftYPos - kFrontRightYPos;

}
