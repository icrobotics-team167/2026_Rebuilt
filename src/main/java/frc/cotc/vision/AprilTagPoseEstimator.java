// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

public class AprilTagPoseEstimator {
  public static final AprilTagFieldLayout tagLayout;

  protected static final HashMap<String, Transform3d> cameraTransforms = new HashMap<>();

  static {
    tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    cameraTransforms.put(
        "FrontLeft",
        new Transform3d(
            Units.inchesToMeters(22 / 2 - 1.5),
            Units.inchesToMeters(32 / 2 + .125),
            Units.inchesToMeters(8.375),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-55.5))));
    cameraTransforms.put(
        "FrontRight",
        new Transform3d(
            Units.inchesToMeters(22 / 2 - 1.5),
            -Units.inchesToMeters(32 / 2 + .125),
            Units.inchesToMeters(8.375),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(55.5))));
  }

  private final PhotonPoseEstimator poseEstimator;

  private final AprilTagPoseEstimatorIO io;
  private final AprilTagPoseEstimatorIO.AprilTagPoseEstimatorIOInputs inputs =
      new AprilTagPoseEstimatorIO.AprilTagPoseEstimatorIOInputs();

  @FunctionalInterface
  public interface VisionEstimateConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }

  private final String name;

  public AprilTagPoseEstimator(String name) {
    io =
        Robot.mode == Robot.Mode.REPLAY
            ? new AprilTagPoseEstimatorIO() {}
            : new AprilTagPoseEstimatorIOPhoton(name);
    this.name = name;
    poseEstimator =
        new PhotonPoseEstimator(
            tagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraTransforms.get(name));
  }

  // data filtering system that Jaynou added
  private boolean isValidPose(EstimatedRobotPose est, Pose2d currentPoseEstimate) {
    Pose3d pose3d = est.estimatedPose;
    Pose2d pose2d = pose3d.toPose2d();

    // floor and sky clip checking
    if (pose3d.getZ() < -0.3 || pose3d.getZ() > 0.8) return false;

    // out of bounds clip checking
    if (pose2d.getX() < 0 || pose2d.getX() > tagLayout.getFieldLength()) return false;
    if (pose2d.getY() < 0 || pose2d.getY() > tagLayout.getFieldWidth()) return false;

    // odometry check (inspired by 2025 vision)
    return currentPoseEstimate == null
        || !(pose2d.getTranslation().getDistance(currentPoseEstimate.getTranslation()) > 0.25
            || pose2d.getRotation().getDegrees() - currentPoseEstimate.getRotation().getDegrees()
                > 2);
  }

  public void update(VisionEstimateConsumer estimateConsumer, Pose2d currentPoseEstimate) {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTags/" + name, inputs);

    for (var result : inputs.results) {
      poseEstimator
          .update(result)
          .ifPresent(
              poseEstimate -> {
                // data filtering
                if (!isValidPose(poseEstimate, currentPoseEstimate)) {
                  return;
                }

                estimateConsumer.accept(
                    poseEstimate.estimatedPose.toPose2d(),
                    result.getTimestampSeconds(),
                    VecBuilder.fill(
                        0.9 / poseEstimate.targetsUsed.size(),
                        0.9 / poseEstimate.targetsUsed.size(),
                        0.9 / poseEstimate.targetsUsed.size()));
              });
    }
  }
}
