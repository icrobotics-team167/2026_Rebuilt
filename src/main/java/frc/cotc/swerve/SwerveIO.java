// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  class SwerveIOConstants {
    double robotMassKg = 50;
    double robotMOIKgMetersSquared = 10;

    double trackWidthMeters = 1;
    double trackLengthMeters = 1;
    double wheelRadiusMeters = Units.inchesToMeters(2);
    DCMotor driveMotor = DCMotor.getKrakenX60Foc(1).withReduction(6.75);
    double slipCurrentAmps = 90;
    double supplyCurrentLimitAmps = 40;

    double maxSteerSpeedRadPerSec =
        DCMotor.getKrakenX60(1).withReduction(150.0 / 7).freeSpeedRadPerSec;
  }

  @AutoLog
  class SwerveIOInputs {
    // TODO: Fill
    SwerveModuleState[] currentStates;
    Rotation2d gyroYaw;

    OdometryFrame[] odometryFrames;
  }

  record OdometryFrame(
      SwerveModulePosition[] positions, Rotation2d gyroYaw, double timestampSeconds) {}

  default SwerveIOConstantsAutoLogged getConstants() {
    return new SwerveIOConstantsAutoLogged();
  }

  default void updateInputs(SwerveIOInputs inputs) {}

  default void drive(CornySetpointGenerator.Setpoint setpoint) {}
}
