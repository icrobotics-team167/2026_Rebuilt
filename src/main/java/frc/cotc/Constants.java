// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import frc.cotc.swerve.TunerConstants;

public class Constants {
  private Constants() {}

  public static final String MOTOR_DISCONNECT_ALERT_GROUP = "Motor Disconnects";

  public static final double offset = Units.inchesToMeters(2.625);
  public static final double bumperThickness = Units.inchesToMeters(2.5);
  public static final double trackWidth =
      TunerConstants.kFrontLeftYPos.minus(TunerConstants.kBackLeftYPos).in(Meters);
  public static final double trackLength =
      TunerConstants.kFrontLeftYPos.minus(TunerConstants.kFrontRightYPos).in(Meters);
  public static final double frameWidth = trackWidth + offset * 2;
  public static final double frameLength = trackLength + offset * 2;
  public static final double bumperWidth = frameWidth + bumperThickness * 2;
  public static final double bumperLength = frameLength + bumperThickness * 2;
}
