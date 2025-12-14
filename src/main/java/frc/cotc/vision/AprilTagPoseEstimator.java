// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.cotc.vision.AprilTagPoseEstimatorIO.AprilTagPoseEstimatorIOInputs;
import frc.cotc.vision.AprilTagPoseEstimatorIO.AprilTagPoseEstimatorIOInputs.AprilTagPoseEstimate;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AprilTagPoseEstimator {
  public static final AprilTagFieldLayout tagLayout;

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
  }

  private final AprilTagPoseEstimatorIO io;
  private final AprilTagPoseEstimatorIOInputs inputs = new AprilTagPoseEstimatorIOInputs();
  public final Transform3d robotToCameraTransform;
  private final Transform2d cameraToRobotTransform2d;

  private final GyroYawGetter gyroYawGetter;
  private final Supplier<Pose2d> currentPoseEstimateSupplier;

  public final String name;

  public record IO(AprilTagPoseEstimatorIO io, String name) {}

  @FunctionalInterface
  public interface GyroYawGetter {
    Rotation2d get(double timestamp);
  }

  public AprilTagPoseEstimator(
      AprilTagPoseEstimatorIO io,
      String name,
      GyroYawGetter gyroYawGetter,
      Supplier<Pose2d> currentPoseEstimateSupplier) {
    this.io = io;
    this.name = name;

    var constants = io.getConstants();
    Logger.processInputs("Vision/" + name + "/CONSTANTS", constants);
    robotToCameraTransform = constants.robotToCameraTransform;
    var cameraToRobotTransform3d = constants.robotToCameraTransform.inverse();
    cameraToRobotTransform2d =
        new Transform2d(
            cameraToRobotTransform3d.getTranslation().toTranslation2d(),
            cameraToRobotTransform3d.getRotation().toRotation2d());
    this.gyroYawGetter = gyroYawGetter;
    this.currentPoseEstimateSupplier = currentPoseEstimateSupplier;
  }

  private final ArrayList<PoseEstimate> estimatesList = new ArrayList<>();
  private final ArrayList<Pose3d> posesUsed = new ArrayList<>();
  private final ArrayList<Pose3d> tagsUsed = new ArrayList<>();

  public PoseEstimate[] poll() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision/" + name, inputs);

    estimatesList.clear();
    posesUsed.clear();
    tagsUsed.clear();
    for (int i = 0; i < inputs.poseEstimates.length; i++) {
      var estimate = inputs.poseEstimates[i];

      switch (estimate.tagsUsed().length) {
        case 0 -> {} // Do nothing
        case 1 -> trigEstimate(estimate);
        default -> solvePnPEstimate(estimate);
      }
    }

    Logger.recordOutput("Vision/" + name + "/Poses Used", posesUsed.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/" + name + "/Tags Used", tagsUsed.toArray(new Pose3d[0]));
    return estimatesList.toArray(new PoseEstimate[0]);
  }

  private void trigEstimate(AprilTagPoseEstimate estimate) {
    var tag = estimate.tagsUsed()[0];

    // Fall back to SolvePnP if key data is missing (position of tag and gyro yaw)
    var tagPoseOptional = tagLayout.getTagPose(tag.id());
    var gyroYaw = gyroYawGetter.get(estimate.timestamp());
    if (tagPoseOptional.isEmpty() || gyroYaw == null) {
      solvePnPEstimate(estimate);
      return;
    }
    var tagPose = tagPoseOptional.get();

    // Algorithm adapted from 6328 Mechanical Advantage
    // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
    var cameraToTagTranslation =
        new Pose3d(
                Translation3d.kZero,
                new Rotation3d(
                    0, Units.degreesToRadians(-tag.ty()), Units.degreesToRadians(-tag.tx())))
            .transformBy(
                new Transform3d(new Translation3d(tag.distanceToCamera(), 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, robotToCameraTransform.getRotation().getY(), 0))
            .toTranslation2d();
    var cameraToTagRotation =
        gyroYaw.plus(
            robotToCameraTransform
                .getRotation()
                .toRotation2d()
                .plus(cameraToTagTranslation.getAngle()));

    var cameraTranslation =
        new Pose2d(
                tagPose.getTranslation().toTranslation2d(),
                cameraToTagRotation.plus(Rotation2d.kPi))
            .transformBy(new Transform2d(cameraToTagTranslation.getNorm(), 0, Rotation2d.kZero))
            .getTranslation();
    var robotPose =
        new Pose2d(
                cameraTranslation,
                gyroYaw.plus(robotToCameraTransform.getRotation().toRotation2d()))
            .transformBy(cameraToRobotTransform2d);
    robotPose = new Pose2d(robotPose.getTranslation(), gyroYaw);

    // Obviously bad data falls back to SolvePnP
    if (robotPose.getX() < 0 || robotPose.getX() > tagLayout.getFieldLength()) {
      solvePnPEstimate(estimate);
      return;
    }
    if (robotPose.getY() < 0 || robotPose.getY() > tagLayout.getFieldWidth()) {
      solvePnPEstimate(estimate);
      return;
    }

    // If it's too far off the pose estimate that's already in, fall back to SolvePnP
    // Prevents bad data from bad initial conditions from affecting estimates
    var delta = robotPose.minus(currentPoseEstimateSupplier.get());
    if (delta.getTranslation().getNorm() > .025 || delta.getRotation().getDegrees() > 2) {
      solvePnPEstimate(estimate);
      return;
    }

    tagsUsed.add(tagPose);
    posesUsed.add(new Pose3d(robotPose));

    // Heavy distrust compared multi-tag SolvePnp, due to the inherent lack of information
    // usable in the solve
    var stdDev = .3 * tag.distanceToCamera() * tag.distanceToCamera();

    estimatesList.add(new PoseEstimate(robotPose, estimate.timestamp(), stdDev, 15));
  }

  private void solvePnPEstimate(AprilTagPoseEstimate estimate) {
    // Filter out obviously bad data
    if (Math.abs(estimate.robotPoseEstimate().getZ()) > .025) {
      return;
    }
    if (estimate.robotPoseEstimate().getX() < 0
        || estimate.robotPoseEstimate().getX() > tagLayout.getFieldLength()) {
      return;
    }
    if (estimate.robotPoseEstimate().getY() < 0
        || estimate.robotPoseEstimate().getY() > tagLayout.getFieldWidth()) {
      return;
    }
    double maxAmbiguity = .2;
    if (estimate.tagsUsed().length == 1 && estimate.tagsUsed()[0].ambiguity() > maxAmbiguity) {
      return;
    }

    double translationalScoresSum = 0;
    double angularScoresSum = 0;
    for (var tag : estimate.tagsUsed()) {
      tagsUsed.add(tag.location());
      var tagDistance = tag.distanceToCamera();

      translationalScoresSum += .3 * tagDistance * tagDistance;
      angularScoresSum += .15 * tagDistance * tagDistance;
    }

    // Heavily distrust single tag observations
    if (estimate.tagsUsed().length == 1) {
      var scale = estimate.tagsUsed()[0].ambiguity() / maxAmbiguity;
      translationalScoresSum *= MathUtil.interpolate(10, 50, scale);
      angularScoresSum *= MathUtil.interpolate(25, 100, scale);
    }

    var translationalDivisor = Math.pow(estimate.tagsUsed().length, 1.5);
    var angularDivisor = Math.pow(estimate.tagsUsed().length, 3);

    posesUsed.add(estimate.robotPoseEstimate());

    estimatesList.add(
        new PoseEstimate(
            estimate.robotPoseEstimate().toPose2d(),
            estimate.timestamp(),
            translationalScoresSum / translationalDivisor,
            angularScoresSum / angularDivisor));
  }

  public record PoseEstimate(
      Pose2d pose, double timestamp, double translationalStdDevs, double rotationalStdDevs) {}
}
