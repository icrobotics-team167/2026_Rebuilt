// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.cotc.vision.AprilTagPoseEstimator;

public class Constants {
  private Constants() {}

  public static final String MOTOR_DISCONNECT_ALERT_GROUP = "Motor Disconnects";

  public static final double FIELD_LENGTH_METERS = AprilTagPoseEstimator.tagLayout.getFieldLength();
  public static final double FIELD_WIDTH_METERS = AprilTagPoseEstimator.tagLayout.getFieldWidth();

  public static final Translation2d BLUE_HUB_LOCATION =
      new Translation2d(Units.inchesToMeters(158.6 + (47.0 / 2)), FIELD_WIDTH_METERS / 2);
  public static final Translation2d RED_HUB_LOCATION =
      new Translation2d(FIELD_LENGTH_METERS - BLUE_HUB_LOCATION.getX(), BLUE_HUB_LOCATION.getY());
}
