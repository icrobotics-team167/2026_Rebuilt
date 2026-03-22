// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import frc.cotc.Robot;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagPoseEstimatorIOPhoton implements AprilTagPoseEstimatorIO {
  private static VisionSystemSim sim;

  static {
    if (Robot.mode == Robot.Mode.SIM) {
      sim = new VisionSystemSim("sim");
      sim.addAprilTags(AprilTagPoseEstimator.tagLayout);
    }
  }

  private final PhotonCamera camera;

  public AprilTagPoseEstimatorIOPhoton(String name) {
    camera = new PhotonCamera(name);
    if (Robot.mode == Robot.Mode.SIM) {
      var cameraCharacteristics = AprilTagPoseEstimator.cameraCharacteristics.get(name);
      var cameraSim =
          new PhotonCameraSim(
              camera,
              new SimCameraProperties()
                  .setCalibration(
                      1280,
                      800,
                      cameraCharacteristics.cameraMatrix(),
                      cameraCharacteristics.distortionCoefficients())
                  .setCalibError(
                      cameraCharacteristics.calibErrorPx(), cameraCharacteristics.errorStdDevPx())
                  .setAvgLatencyMs(10)
                  .setLatencyStdDevMs(1)
                  .setFPS(45)
                  .setExposureTimeMs(5),
              AprilTagPoseEstimator.tagLayout);
      cameraSim.enableDrawWireframe(false);
      cameraSim.enableRawStream(false);
      cameraSim.enableProcessedStream(false);
      sim.addCamera(cameraSim, cameraCharacteristics.robotToCamera());
    }
  }

  public static void updateSim() {
    sim.update(Robot.groundTruthPoseSupplier.get());
  }

  public static void resetSim() {
    sim.resetRobotPose(Robot.groundTruthPoseSupplier.get());
    sim.resetCameraTransforms();
  }

  @Override
  public void updateInputs(AprilTagPoseEstimatorIOInputs inputs) {
    inputs.results = camera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);
  }
}
