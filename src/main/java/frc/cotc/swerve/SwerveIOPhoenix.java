// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.cotc.Robot;
import java.util.ArrayList;

public class SwerveIOPhoenix implements SwerveIO {
  private final TunerConstants.TunerSwerveDrivetrain drivetrain =
      new TunerConstants.TunerSwerveDrivetrain(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  private final ArrayList<SwerveDrivetrain.SwerveDriveState> statesList = new ArrayList<>();

  public SwerveIOPhoenix() {
    drivetrain.registerTelemetry(
        (state) -> {
          var clonedState = state.clone();
          synchronized (statesList) {
            statesList.add(clonedState);
          }
        });
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    if (Robot.mode == Robot.Mode.SIM) {
      drivetrain.updateSimState(.02, 12);
    }
    SwerveDrivetrain.SwerveDriveState[] statesArray;
    synchronized (statesList) {
      statesArray = statesList.toArray(new SwerveDrivetrain.SwerveDriveState[0]);
    }
    inputs.odoStates = statesArray;
    inputs.currentState = drivetrain.getState();
  }

  @Override
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    drivetrain.addVisionMeasurement(visionPose, timestamp, stdDevs);
  }

  private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();

  @Override
  public void drive(ChassisSpeeds speeds) {
    drivetrain.setControl(
        fieldCentricRequest
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }
}
