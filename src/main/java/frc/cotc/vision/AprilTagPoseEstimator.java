// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;

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
import frc.cotc.FieldConstants;
import frc.cotc.Robot;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.timesync.TimeSyncSingleton;

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

    // Cameras got swapped around and I CBA to rename them
    // TODO: Rename and recalibrate
    cameraCharacteristics.put(
        "Back",
        new CameraCharacteristics(
            new Transform3d(
                Units.inchesToMeters(-11 + 0.5),
                Units.inchesToMeters(-16.125 + 2.5),
                Units.inchesToMeters(21.125 - .75),
                new Rotation3d(0, Units.degreesToRadians(-15), Math.PI)),
            MatBuilder.fill(
                Nat.N3(),
                Nat.N3(),
                911.6805417738924,
                0.0,
                656.1304695468573,
                0.0,
                912.1546553469605,
                444.98299255079814,
                0.0,
                0.0,
                1.0),
            VecBuilder.fill(
                .04795119918130085,
                -0.062419105839332197,
                0.0011565212857560978,
                -5.526702632857694E-4,
                0.004938645098226732,
                -0.0019122643292644806,
                0.007284330071940078,
                0.003395780379915023),
            .5,
            .5));
    cameraCharacteristics.put(
        "Right",
        new CameraCharacteristics(
            new Transform3d(
                Units.inchesToMeters(-11 + 1.25),
                Units.inchesToMeters(-16.125 + 0),
                Units.inchesToMeters(21.125 - 0.75),
                new Rotation3d(
                    Units.degreesToRadians(0), Units.degreesToRadians(-15), -Math.PI / 2)),
            MatBuilder.fill(
                Nat.N3(),
                Nat.N3(),
                913.5769706149941,
                0.0,
                652.4805585543866,
                0.0,
                913.6880733305309,
                438.42963573676263,
                0.0,
                0.0,
                1.0),
            VecBuilder.fill(
                0.043640621444117955,
                -0.0570929951207751,
                -8.603760109188229E-4,
                -1.4658776672674253E-4,
                -0.010502255926284516,
                -0.0020113855907111624,
                -6.399691332393489E-5,
                -6.010419737104136E-4),
            .5,
            .5));
    cameraCharacteristics.put(
        "Left",
        new CameraCharacteristics(
            new Transform3d(
                Units.inchesToMeters(-11 + 1.25),
                Units.inchesToMeters(16.125 + -0.5),
                Units.inchesToMeters(21.125 - 0.75),
                new Rotation3d(0, Units.degreesToRadians(-15), Math.PI / 2)),
            MatBuilder.fill(
                Nat.N3(),
                Nat.N3(),
                903.055394105631,
                0.0,
                626.4504806409194,
                0.0,
                902.8996397233539,
                441.2614581199893,
                0.0,
                0.0,
                1.0),
            VecBuilder.fill(
                0.0485892167270983,
                -0.06647373800197513,
                -4.5133828078128144E-4,
                -8.540950502923184E-4,
                0.004362226375534471,
                -0.0018703373716495869,
                0.003832307751646421,
                2.560923932909948E-4),
            .5,
            .5));
    cameraCharacteristics.put(
        "Front",
        new CameraCharacteristics(
            new Transform3d(
                Units.inchesToMeters(-11 + 3.25),
                Units.inchesToMeters(16.125 - 1.75),
                Units.inchesToMeters(21.125 - 0.75),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), 0)),
            MatBuilder.fill(
                Nat.N3(),
                Nat.N3(),
                910.8083030803828,
                0.0,
                664.4718645147575,
                0.0,
                911.3001066557107,
                419.47133406032316,
                0.0,
                0.0,
                1.0),
            VecBuilder.fill(
                0.042369983378026846,
                -0.06709932133277403,
                -0.0012209494764653978,
                -3.283769547303972E-4,
                0.008945573196069931,
                -0.001581364863391461,
                0.0028575430831169787,
                -2.61093847726435E-4),
            .5,
            .5));
  }

  private final PhotonPoseEstimator poseEstimator;

  private final AprilTagPoseEstimatorIO io;
  private final AprilTagPoseEstimatorIO.AprilTagPoseEstimatorIOInputs inputs =
      new AprilTagPoseEstimatorIO.AprilTagPoseEstimatorIOInputs();

  private final String name;
  private final Transform3d robotToCamera;

  public AprilTagPoseEstimator(String name) {
    io =
        Robot.mode == Robot.Mode.REPLAY
            ? new AprilTagPoseEstimatorIO() {}
            : new AprilTagPoseEstimatorIOPhoton(name);
    this.name = name;
    var characteristics = cameraCharacteristics.get(name);
    robotToCamera = characteristics.robotToCamera();
    poseEstimator = new PhotonPoseEstimator(tagLayout, characteristics.robotToCamera());
    TimeSyncSingleton.load();
  }

  public void addPoseData(double timestampSeconds, Pose2d pose) {
    poseEstimator.addHeadingData(timestampSeconds, pose.getRotation());
  }

  public record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

  private Consumer<VisionMeasurement> estimateConsumer;

  public void setEstimateConsumer(Consumer<VisionMeasurement> estimateConsumer) {
    this.estimateConsumer = estimateConsumer;
  }

  private final ArrayList<Pose3d> acceptedPoses = new ArrayList<>();
  private final ArrayList<Pose3d> rejectedPoses = new ArrayList<>();
  private final ArrayList<Pose3d> tagsSeen = new ArrayList<>();

  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTags/" + name, inputs);

    acceptedPoses.clear();
    rejectedPoses.clear();
    tagsSeen.clear();

    for (var result : inputs.results) {
      switch (result.targets.size()) {
        case 0 -> {} // This shouldn't happen, but just in case
        case 1 -> poseEstimator.estimateLowestAmbiguityPose(result).ifPresent(this::addMeasurement);
        default -> poseEstimator.estimateCoprocMultiTagPose(result).ifPresent(this::addMeasurement);
      }
    }

    Logger.recordOutput("Vision/" + name + "/Rejected poses", rejectedPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/" + name + "/Accepted poses", acceptedPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/" + name + "/Tags seen", tagsSeen.toArray(new Pose3d[0]));
  }

  private void addMeasurement(EstimatedRobotPose est) {
    var pose = est.estimatedPose;
    if (pose.getX() < 0 || pose.getX() > FieldConstants.fieldLength) {
      rejectedPoses.add(pose);
      return;
    }
    if (pose.getY() < 0 || pose.getY() > FieldConstants.fieldWidth) {
      rejectedPoses.add(pose);
      return;
    }
    if (pose.getZ() < -0.2 || pose.getZ() > 0.3) {
      rejectedPoses.add(pose);
      return;
    }
    if (Math.hypot(pose.getRotation().getX(), pose.getRotation().getY())
        > Units.degreesToRadians(20)) {
      rejectedPoses.add(pose);
      return;
    }
    if (est.strategy == LOWEST_AMBIGUITY && est.targetsUsed.get(0).poseAmbiguity > 0.2) {
      rejectedPoses.add(pose);
      return;
    }
    acceptedPoses.add(pose);
    for (var target : est.targetsUsed) {
      tagsSeen.add(pose.plus(robotToCamera).plus(target.getBestCameraToTarget()));
    }
    estimateConsumer.accept(
        new VisionMeasurement(
            pose.toPose2d(),
            est.timestampSeconds,
            switch (est.strategy) {
              case LOWEST_AMBIGUITY -> {
                var tagDistance =
                    est.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm();
                yield VecBuilder.fill(
                    0.1 * Math.pow(tagDistance, 2.5),
                    0.1 * Math.pow(tagDistance, 2.5),
                    0.1 * Math.pow(tagDistance, 2.5));
              }
              case MULTI_TAG_PNP_ON_COPROCESSOR -> {
                var avgTagDistance = 0.0;
                for (var target : est.targetsUsed) {
                  avgTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                }
                avgTagDistance /= est.targetsUsed.size();
                yield VecBuilder.fill(
                    0.5 * Math.pow(avgTagDistance, 2) / est.targetsUsed.size(),
                    0.5 * Math.pow(avgTagDistance, 2) / est.targetsUsed.size(),
                    1 * Math.pow(avgTagDistance, 2) / est.targetsUsed.size());
              }
              default -> VecBuilder.fill(0.9, 0.9, 0.9);
            }));
  }
}
