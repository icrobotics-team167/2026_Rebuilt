// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface SwerveIO {
  // @AutoLog
  class SwerveIOInputs {
    public SwerveDrivetrain.SwerveDriveState[] odoStates;
    public SwerveDrivetrain.SwerveDriveState currentState;
  }

  default void updateInputs(SwerveIOInputs inputs) {}

  default void drive(ChassisSpeeds speeds) {}

  default void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
