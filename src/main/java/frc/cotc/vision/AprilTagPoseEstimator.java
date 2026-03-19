// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;
import java.util.HashMap;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.timesync.TimeSyncSingleton;

@SuppressWarnings("OptionalUsedAsFieldOrParameterType")
public class AprilTagPoseEstimator {
  public static final AprilTagFieldLayout tagLayout;

  protected static final HashMap<String, CameraCharacteristics> cameraCharacteristics =
      new HashMap<>();

  protected record CameraCharacteristics(
      Transform3d robotToCamera,
      Matrix<N3, N3> cameraMatrix,
      Matrix<N8, N1> distortionCoefficients,
      double calibErrorPx,
      double errorStdDevPx) {}

  static {
    tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    cameraCharacteristics.put(
        "BackLeft",
        new CameraCharacteristics(
            new Transform3d(
                -Units.inchesToMeters(22.0 / 2 - 2),
                Units.inchesToMeters(32.0 / 2 - 2),
                Units.inchesToMeters(28.75),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(130))),
            MatBuilder.fill(
                Nat.N3(),
                Nat.N3(),
                895.7697683626751,
                0.0,
                659.6060292695324,
                0.0,
                896.0068506346151,
                451.10507981030486,
                0.0,
                0.0,
                1.0),
            VecBuilder.fill(0, 0, 0, 0, 0, 0, 0, 0),
            0.5,
            0.1));
    cameraCharacteristics.put(
        "BackRight",
        new CameraCharacteristics(
            new Transform3d(
                -Units.inchesToMeters(22.0 / 2 - 2),
                -Units.inchesToMeters(32.0 / 2 - 2),
                Units.inchesToMeters(18.75),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-135))),
            MatBuilder.fill(
                Nat.N3(),
                Nat.N3(),
                895.7697683626751,
                0.0,
                659.6060292695324,
                0.0,
                896.0068506346151,
                451.10507981030486,
                0.0,
                0.0,
                1.0),
            VecBuilder.fill(0, 0, 0, 0, 0, 0, 0, 0),
            0.5,
            0.1));
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

  final Optional<Matrix<N3, N3>> optionalCameraMatrix;
  final Optional<Matrix<N8, N1>> optionalDistortionCoefficients;

  public AprilTagPoseEstimator(String name) {
    io =
        Robot.mode == Robot.Mode.REPLAY
            ? new AprilTagPoseEstimatorIO() {}
            : new AprilTagPoseEstimatorIOPhoton(name);
    this.name = name;
    var characteristics = cameraCharacteristics.get(name);
    optionalCameraMatrix = Optional.of(characteristics.cameraMatrix);
    optionalDistortionCoefficients = Optional.of(characteristics.distortionCoefficients);
    poseEstimator =
        new PhotonPoseEstimator(
            tagLayout,
            PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP,
            characteristics.robotToCamera());
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    TimeSyncSingleton.load();
  }

  // data filtering system that Jaynou added
  private boolean isValidPose(EstimatedRobotPose est) {
    var pose = est.estimatedPose;

    // floor and sky clip checking
    if (pose.getZ() < -0.3 || pose.getZ() > 0.8) return false;

    // out of bounds clip checking
    if (pose.getX() < 0 || pose.getX() > tagLayout.getFieldLength()) return false;
    if (pose.getY() < 0 || pose.getY() > tagLayout.getFieldWidth()) return false;

    return true;
  }

  public void addHeadingData(double timestampSeconds, Rotation2d heading) {
    poseEstimator.addHeadingData(timestampSeconds, heading);
  }

  private boolean enabled = false;

  public void setEnabled() {
    poseEstimator.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    enabled = true;
  }

  public void setDisabled() {
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    enabled = false;
  }

  public void update(VisionEstimateConsumer estimateConsumer) {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTags/" + name, inputs);

    for (var result : inputs.results) {
      poseEstimator
          .update(
              result,
              optionalCameraMatrix,
              optionalDistortionCoefficients,
              Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(!enabled, 1.0)))
          .ifPresent(
              poseEstimate -> {
                // data filtering
                if (!isValidPose(poseEstimate)) {
                  return;
                }

                double translationalScoresSum = 0;
                double angularScoresSum = 0;
                for (var tag : poseEstimate.targetsUsed) {
                  var tagDistance = tag.bestCameraToTarget.getTranslation().getNorm();

                  translationalScoresSum += .2 * tagDistance * tagDistance;
                  angularScoresSum += .2 * tagDistance * tagDistance;
                }

                var translationalDivisor = Math.pow(poseEstimate.targetsUsed.size(), 2);
                var angularDivisor = Math.pow(poseEstimate.targetsUsed.size(), 3);

                estimateConsumer.accept(
                    poseEstimate.estimatedPose.toPose2d(),
                    result.getTimestampSeconds(),
                    VecBuilder.fill(
                        translationalScoresSum / translationalDivisor,
                        translationalScoresSum / translationalDivisor,
                        poseEstimate.strategy
                                == PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
                            ? Double.POSITIVE_INFINITY
                            : angularScoresSum / angularDivisor));
              });
    }
  }
}
