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
import java.util.ArrayList;
import java.util.HashMap;
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
        "Front",
        new CameraCharacteristics(
            new Transform3d(
                Units.inchesToMeters(-11 + 2.4375),
                Units.inchesToMeters(16 - 1.3125),
                Units.inchesToMeters(28.25),
                new Rotation3d(0, Units.degreesToRadians(-15), 0)),
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
            .31,
            .2));
    cameraCharacteristics.put(
        "Right",
        new CameraCharacteristics(
            new Transform3d(
                Units.inchesToMeters(-11 + 1.3125),
                Units.inchesToMeters(16 - 2.875),
                Units.inchesToMeters(28.25),
                new Rotation3d(
                    Units.degreesToRadians(5), Units.degreesToRadians(-15), -Math.PI / 2)),
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
            .44,
            .2));
    cameraCharacteristics.put(
        "Back",
        new CameraCharacteristics(
            new Transform3d(
                Units.inchesToMeters(-11 + 0.75),
                Units.inchesToMeters(-16 + 1.25),
                Units.inchesToMeters(28.25),
                new Rotation3d(0, Units.degreesToRadians(-15), Math.PI)),
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
            .43,
            .2));
    cameraCharacteristics.put(
        "Left",
        new CameraCharacteristics(
            new Transform3d(
                Units.inchesToMeters(-11 + 1.625),
                Units.inchesToMeters(-16 + 2.875),
                Units.inchesToMeters(28.25),
                new Rotation3d(
                    Units.degreesToRadians(5), Units.degreesToRadians(-15), Math.PI / 2)),
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
            .35,
            .2));
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
    var characteristics = cameraCharacteristics.get(name);
    poseEstimator =
        new PhotonPoseEstimator(
            tagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            characteristics.robotToCamera());
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    TimeSyncSingleton.load();
  }

  // data filtering system that Jaynou added
  private boolean isValidPose(EstimatedRobotPose est) {
    var pose = est.estimatedPose;

    // floor and sky clip checking
    if (pose.getZ() < -0.1 || pose.getZ() > 0.05) return false;

    if (Math.abs(pose.getRotation().getX()) > Units.degreesToRadians(10)) return false;
    if (Math.abs(pose.getRotation().getY()) > Units.degreesToRadians(10)) return false;

    // out of bounds clip checking
    if (pose.getX() < 0 || pose.getX() > tagLayout.getFieldLength()) return false;
    if (pose.getY() < 0 || pose.getY() > tagLayout.getFieldWidth()) return false;

    return true;
  }

  public void addPoseData(double timestampSeconds, Pose2d pose) {
    poseEstimator.setReferencePose(pose);
    poseEstimator.addHeadingData(timestampSeconds, pose.getRotation());
  }

  public void setEnabled() {
    poseEstimator.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  public void setDisabled() {
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
  }

  public void update(VisionEstimateConsumer estimateConsumer) {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTags/" + name, inputs);

    var acceptedPoses = new ArrayList<Pose3d>();
    var rejectedPoses = new ArrayList<Pose3d>();

    for (var result : inputs.results) {
      poseEstimator
          .update(result)
          .ifPresent(
              poseEstimate -> {
                // data filtering
                if (!isValidPose(poseEstimate)) {
                  rejectedPoses.add(poseEstimate.estimatedPose);
                  return;
                }
                acceptedPoses.add(poseEstimate.estimatedPose);

                double translationalScoresSum = 0;
                double angularScoresSum = 0;
                for (var tag : poseEstimate.targetsUsed) {
                  var tagDistance = tag.bestCameraToTarget.getTranslation().getNorm();

                  translationalScoresSum += .9 * Math.pow(tagDistance, 2.5);
                  angularScoresSum += .6 * Math.pow(tagDistance, 2.5);
                }

                var translationalDivisor = Math.pow(poseEstimate.targetsUsed.size(), 1.5);
                var angularDivisor = Math.pow(poseEstimate.targetsUsed.size(), 1.5);

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

    Logger.recordOutput("Vision/" + name + "/Rejected poses", rejectedPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/" + name + "/Accepted poses", acceptedPoses.toArray(new Pose3d[0]));
  }
}
