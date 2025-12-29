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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.cotc.Robot;
import java.io.IOException;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

public class AprilTagPoseEstimator {
  public static final AprilTagFieldLayout tagLayout;

  protected static final HashMap<String, Transform3d> cameraTransforms = new HashMap<>();

  static {
    AprilTagFieldLayout fieldLayout;
    try {
      fieldLayout =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory() + "/2025-reefscape-welded-reefonly.json");
    } catch (IOException e) {
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }
    tagLayout = fieldLayout;

    cameraTransforms.put(
        "FrontLeft",
        new Transform3d(
            Units.inchesToMeters(22.75 / 2 - 1.5),
            Units.inchesToMeters(22.75 / 2 + .125),
            Units.inchesToMeters(8.375),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-30))));
    cameraTransforms.put(
        "FrontRight",
        new Transform3d(
            Units.inchesToMeters(22.75 / 2 - 1.5),
            -Units.inchesToMeters(22.75 / 2 + .125),
            Units.inchesToMeters(8.375),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(30))));
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

  public void update(VisionEstimateConsumer estimateConsumer) {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTags/" + name, inputs);
    for (var result : inputs.results) {
      poseEstimator
          .update(result)
          .ifPresent(
              poseEstimate ->
                  estimateConsumer.accept(
                      poseEstimate.estimatedPose.toPose2d(),
                      result.getTimestampSeconds(),
                      VecBuilder.fill(
                          .9 / poseEstimate.targetsUsed.size(),
                          .9 / poseEstimate.targetsUsed.size(),
                          .9 / poseEstimate.targetsUsed.size())));
    }
  }
}
