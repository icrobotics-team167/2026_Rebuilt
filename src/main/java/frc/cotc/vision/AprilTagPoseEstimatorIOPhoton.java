// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.cotc.Robot;
import frc.cotc.vision.AprilTagPoseEstimatorIO.AprilTagPoseEstimatorIOInputs.AprilTagPoseEstimate;
import frc.cotc.vision.AprilTagPoseEstimatorIO.AprilTagPoseEstimatorIOInputs.AprilTagPoseEstimate.AprilTag;
import java.util.ArrayList;
import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagPoseEstimatorIOPhoton implements AprilTagPoseEstimatorIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final AprilTagPoseEstimatorIOConstantsAutoLogged constants;

  public AprilTagPoseEstimatorIOPhoton(String name, Transform3d robotToCameraTransform) {
    camera = new PhotonCamera(name);
    poseEstimator =
        new PhotonPoseEstimator(
            AprilTagPoseEstimator.tagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCameraTransform);

    constants = new AprilTagPoseEstimatorIOConstantsAutoLogged();
    constants.robotToCameraTransform = robotToCameraTransform;

    if (Robot.isSimulation()) {
      Sim.addCamera(camera, robotToCameraTransform);
    }
  }

  private final ArrayList<AprilTagPoseEstimate> estimatesList = new ArrayList<>();

  @Override
  public void updateInputs(AprilTagPoseEstimatorIOInputs inputs) {
    var results = camera.getAllUnreadResults();

    for (var result : results) {
      poseEstimator
          .update(result)
          .ifPresent(
              estimate -> {
                var tagsUsed = new AprilTag[estimate.targetsUsed.size()];
                for (int i = 0; i < tagsUsed.length; i++) {
                  var tag = estimate.targetsUsed.get(i);
                  tagsUsed[i] =
                      new AprilTag(
                          estimate
                              .estimatedPose
                              .plus(constants.robotToCameraTransform)
                              .plus(tag.bestCameraToTarget),
                          tag.fiducialId,
                          tag.bestCameraToTarget.getTranslation().getNorm(),
                          tag.yaw,
                          tag.pitch,
                          tag.poseAmbiguity);
                }
                estimatesList.add(
                    new AprilTagPoseEstimate(
                        estimate.estimatedPose, estimate.timestampSeconds, tagsUsed));
              });
    }

    inputs.poseEstimates = estimatesList.toArray(new AprilTagPoseEstimate[0]);
    estimatesList.clear();
    inputs.dataCount = inputs.poseEstimates.length;
  }

  @Override
  public AprilTagPoseEstimatorIOConstantsAutoLogged getConstants() {
    return constants;
  }

  public static class Sim {
    private static VisionSystemSim visionSystemSim;

    static void addCamera(PhotonCamera camera, Transform3d transform) {
      if (visionSystemSim == null) {
        visionSystemSim = new VisionSystemSim("main");
        visionSystemSim.addAprilTags(AprilTagPoseEstimator.tagLayout);
      }

      var name = camera.getName();
      SimCameraProperties properties;
      if (propertiesHashMap.containsKey(name)) {
        properties = propertiesHashMap.get(name);
      } else {
        properties = propertiesHashMap.get("None");
      }

      var cameraSim = new PhotonCameraSim(camera, properties);
      cameraSim.enableProcessedStream(false);
      cameraSim.enableRawStream(false);
      cameraSim.enableDrawWireframe(false);
      cameraSim.setMaxSightRange(6);
      visionSystemSim.addCamera(cameraSim, transform);
    }

    public static void update() {
      if (Robot.groundTruthPoseSupplier != null && visionSystemSim != null) {
        visionSystemSim.update(Robot.groundTruthPoseSupplier.get());
      }
    }

    private static final HashMap<String, SimCameraProperties> propertiesHashMap = new HashMap<>();

    static {
      var none = new SimCameraProperties();
      none.setCalibration(1280, 800, Rotation2d.fromDegrees(80));
      none.setCalibError(.3, .1);
      none.setFPS(20);
      none.setExposureTimeMs(10);
      none.setAvgLatencyMs(5);
      none.setLatencyStdDevMs(2);
      propertiesHashMap.put("None", none);

      var frontLeft = new SimCameraProperties();
      frontLeft.setCalibration(
          1280,
          800,
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
          MatBuilder.fill(
              Nat.N8(),
              Nat.N1(),
              0.0333671674342023,
              -0.03416433696651355,
              0.0024363855873020036,
              2.0703698596376814E-4,
              -0.03158846431043834,
              -3.3770550824757396E-4,
              0.0036614076801956123,
              0.002376695294547235));
      frontLeft.setCalibError(.19, .01);
      frontLeft.setFPS(40);
      frontLeft.setExposureTimeMs(5);
      frontLeft.setAvgLatencyMs(25);
      frontLeft.setLatencyStdDevMs(5);
      propertiesHashMap.put("FrontLeft", frontLeft);

      var frontRight = new SimCameraProperties();
      frontRight.setCalibration(
          1280,
          800,
          MatBuilder.fill(
              Nat.N3(),
              Nat.N3(),
              909.6508738125398,
              0.0,
              657.5177321829904,
              0.0,
              909.6394411538412,
              381.92351819122564,
              0.0,
              0.0,
              1.0),
          MatBuilder.fill(
              Nat.N8(),
              Nat.N1(),
              0.04858933721783682,
              -0.08941912550409659,
              -3.1084536948635744E-4,
              4.151606520884132E-4,
              0.03579248740242956,
              3.9706855102229375E-4,
              0.0014672508075894892,
              -0.0029891920125610745));
      frontRight.setCalibError(.2, .01);
      frontRight.setFPS(35);
      frontRight.setExposureTimeMs(5);
      frontRight.setAvgLatencyMs(25);
      frontRight.setLatencyStdDevMs(5);
      propertiesHashMap.put("FrontRight", frontRight);

      var newFrontLeft = new SimCameraProperties();
      newFrontLeft.setCalibration(
          1280,
          800,
          MatBuilder.fill(
              Nat.N3(),
              Nat.N3(),
              915.4325114,
              0.0,
              665.8947488,
              0.0,
              915.7777798,
              420.4957657,
              0.0,
              0.0,
              1.0),
          MatBuilder.fill(
              Nat.N8(),
              Nat.N1(),
              0.03976229713,
              -0.05622955742,
              -0.0001059317984,
              0.0003250262416,
              0.002521360703,
              -0.001455079397,
              0.001565710614,
              -0.0004670496762));
      newFrontLeft.setCalibError(.2, .01);
      newFrontLeft.setFPS(40);
      newFrontLeft.setExposureTimeMs(5);
      newFrontLeft.setAvgLatencyMs(25);
      newFrontLeft.setLatencyStdDevMs(5);
      propertiesHashMap.put("NewFrontLeft", frontLeft);

      var newFrontRight = new SimCameraProperties();
      newFrontRight.setCalibration(
          1280,
          800,
          MatBuilder.fill(
              Nat.N3(),
              Nat.N3(),
              910.3836916,
              0.0,
              644.2284492,
              0.0,
              910.5441685,
              434.9509512,
              0.0,
              0.0,
              1.0),
          MatBuilder.fill(
              Nat.N8(),
              Nat.N1(),
              0.03904592683,
              -0.03707627335,
              -0.0006458818621,
              -0.002026416402,
              -0.0006441872541,
              0.0001673140141,
              0.01441301652,
              0.008230122372));
      newFrontRight.setCalibError(.1, .01);
      newFrontRight.setFPS(35);
      newFrontRight.setExposureTimeMs(5);
      newFrontRight.setAvgLatencyMs(25);
      newFrontRight.setLatencyStdDevMs(5);
      propertiesHashMap.put("NewFrontRight", newFrontRight);
    }
  }
}
