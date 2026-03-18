// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import static frc.cotc.Constants.robotToShooterTransform;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.cotc.FieldConstants;
import frc.cotc.Robot;
import org.littletonrobotics.junction.Logger;

public class SOTM {
  // Shooting on the move will induce drag on the projectile, so compensate for that
  // Time in inverse seconds for the shot's velocity to decay by 1/e times (decay to ~36.8%)
  // TODO: Tune
  private static final double DRAG_CONSTANT_INVERSE_SECONDS = 0.1;

  // Location that the robot should shoot at for passing balls
  private static final Translation2d BLUE_BOTTOM_GROUND_TARGET = new Translation2d(1, 1);
  private static final Translation2d BLUE_TOP_GROUND_TARGET =
      new Translation2d(
          BLUE_BOTTOM_GROUND_TARGET.getX(),
          FieldConstants.fieldWidth - BLUE_BOTTOM_GROUND_TARGET.getY());
  private static final Translation2d RED_BOTTOM_GROUND_TARGET =
      new Translation2d(
          FieldConstants.fieldLength - BLUE_BOTTOM_GROUND_TARGET.getX(),
          BLUE_BOTTOM_GROUND_TARGET.getY());
  private static final Translation2d RED_TOP_GROUND_TARGET =
      new Translation2d(
          RED_BOTTOM_GROUND_TARGET.getX(),
          FieldConstants.fieldWidth - RED_BOTTOM_GROUND_TARGET.getY());

  private static final ShotMap hubShotMap = ShotMap.loadFromDeploy("HubShotMap.json");
  private static final ShotMap groundShotMap = ShotMap.loadFromDeploy("GroundShotMap.json");

  public enum ShotTarget {
    BLUE_HUB(hubShotMap, FieldConstants.Hub.topCenterPoint.toTranslation2d()),
    BLUE_BOTTOM_GROUND(groundShotMap, BLUE_BOTTOM_GROUND_TARGET),
    BLUE_TOP_GROUND(groundShotMap, BLUE_TOP_GROUND_TARGET),
    RED_HUB(hubShotMap, FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()),
    RED_BOTTOM_GROUND(groundShotMap, RED_BOTTOM_GROUND_TARGET),
    RED_TOP_GROUND(groundShotMap, RED_TOP_GROUND_TARGET);

    public final ShotMap map;
    public final Translation2d targetLocation;

    ShotTarget(ShotMap map, Translation2d targetLocation) {
      this.map = map;
      this.targetLocation = targetLocation;
    }
  }

  public record SOTMResult(
      double pitchRad,
      Rotation2d yaw,
      double shotSpeedMetersPerSecond,
      double maxMoveSpeedMetersPerSecond,
      double timeOfFlightSeconds) {}

  public static SOTMResult calculate(
      Pose2d robotPose, ChassisSpeeds fieldChassisSpeeds, ShotTarget shotTarget) {
    // Calculate where the shooter is on the field
    var shooterTranslation = robotPose.plus(robotToShooterTransform).getTranslation();
    // Rotate the robotToShooterTransform by the robot yaw
    var robotCenterToShooter = shooterTranslation.minus(robotPose.getTranslation());

    // Get target location and delta pos from shooter to target
    var targetLocation = shotTarget.targetLocation;

    // Rotation affects the velocity of the shooter, so account for that
    var shooterVx =
        fieldChassisSpeeds.vxMetersPerSecond
            - fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getY();
    var shooterVy =
        fieldChassisSpeeds.vyMetersPerSecond
            + fieldChassisSpeeds.omegaRadiansPerSecond * robotCenterToShooter.getX();

    var map = shotTarget.map;

    final int iterations = 5;
    var iterationsPoses = new Pose2d[iterations + 2];
    var iterationToFs = new double[iterations + 2];
    iterationsPoses[0] =
        new Pose2d(shooterTranslation, targetLocation.minus(shooterTranslation).getAngle());
    // Initial guess using the stationary shot's time of flight
    var baseDistance = targetLocation.getDistance(shooterTranslation);
    var initialGuess = map.get(baseDistance);
    var timeOfFlight = initialGuess.timeOfFlightSeconds();
    iterationToFs[0] = timeOfFlight;
    for (int i = 1; i <= iterations; i++) {
      var a =
          (1 - Math.exp(-DRAG_CONSTANT_INVERSE_SECONDS * timeOfFlight))
              / DRAG_CONSTANT_INVERSE_SECONDS;
      var virtualShooterPos =
          shooterTranslation.plus(
              new Translation2d(shooterVx * timeOfFlight * a, shooterVy * timeOfFlight * a));
      var virtualShooterToTarget = targetLocation.minus(virtualShooterPos); // d(τ)
      iterationsPoses[i] = new Pose2d(virtualShooterPos, virtualShooterToTarget.getAngle());

      var virtualShooterToTargetNorm = virtualShooterToTarget.getNorm(); // D(τ)

      var timeOfFlightDerivative =
          map.getTimeOfFlightDerivative(virtualShooterToTargetNorm); // τ'(D(τ))
      var a_prime = Math.exp(-DRAG_CONSTANT_INVERSE_SECONDS * timeOfFlight); // a'(τ)

      var error = timeOfFlight - map.get(virtualShooterToTargetNorm).timeOfFlightSeconds(); // E(τ)
      var error_prime =
          1
              + a_prime
                  * timeOfFlightDerivative
                  * (virtualShooterToTarget.getX() * shooterVx
                      + virtualShooterToTarget.getY() * shooterVy)
                  / virtualShooterToTargetNorm; // E'(τ)
      timeOfFlight -= error / error_prime;
      iterationToFs[i] = timeOfFlight;
    }
    var finalVirtualShooterPos =
        shooterTranslation.plus(
            new Translation2d(shooterVx * timeOfFlight, shooterVy * timeOfFlight));
    var finalVirtualShooterToTarget = targetLocation.minus(finalVirtualShooterPos);
    iterationsPoses[iterations + 1] =
        new Pose2d(finalVirtualShooterPos, finalVirtualShooterToTarget.getAngle());
    var finalVirtualShooterToTargetDistance = finalVirtualShooterToTarget.getNorm();
    var result = map.get(finalVirtualShooterToTargetDistance);
    iterationToFs[iterations + 1] = result.timeOfFlightSeconds();
    timeOfFlight = result.timeOfFlightSeconds();
    Logger.recordOutput("Shooter/Shot result/Iteration Poses", iterationsPoses);
    Logger.recordOutput("Shooter/Shot result/Iteration ToF seconds", iterationToFs);

    var shotStability =
        Math.abs(
            (iterationToFs[iterations] - iterationToFs[iterations - 1])
                / (iterationToFs[iterations - 1] - iterationToFs[iterations - 2]));
    Logger.recordOutput("Shooter/Shot result/Shot stability", shotStability);
    Logger.recordOutput(
        "Shooter/Shot result/Final virtual shooter pose", iterationsPoses[iterations + 1]);
    Logger.recordOutput("Shooter/Shot result/Result", result);
    Logger.recordOutput("Shooter/Shot result/Distance", finalVirtualShooterToTargetDistance);
    Logger.recordOutput("Shooter/Shot result/Time of flight", timeOfFlight);

    var turretYawAbsolute = finalVirtualShooterToTarget.getAngle();

    if (Robot.isSimulation()) {
      Logger.recordOutput(
          "Shooter/Shot result/Trajectory",
          TrajectoryCalc.simulateShot(
              new Translation3d(
                  shooterTranslation.getX(), shooterTranslation.getY(), Units.inchesToMeters(18)),
              new Translation3d(
                      result.speedMetersPerSec(),
                      new Rotation3d(0, -result.pitchRad(), turretYawAbsolute.getRadians()))
                  .plus(new Translation3d(shooterVx, shooterVy, 0))));
    }
    return new SOTMResult(
        result.pitchRad(),
        turretYawAbsolute,
        result.speedMetersPerSec(),
        initialGuess.speedMetersPerSec() * Math.sin(initialGuess.pitchRad()) / 2,
        timeOfFlight);
  }
}
