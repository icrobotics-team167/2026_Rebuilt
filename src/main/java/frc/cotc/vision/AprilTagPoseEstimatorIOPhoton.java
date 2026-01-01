// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Rotation2d;
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
      var cameraSim =
          new PhotonCameraSim(
              camera,
              new SimCameraProperties()
                  .setCalibration(1280, 800, Rotation2d.fromDegrees(80))
                  .setAvgLatencyMs(10)
                  .setLatencyStdDevMs(1),
              AprilTagPoseEstimator.tagLayout);
      cameraSim.enableDrawWireframe(true);
      cameraSim.enableRawStream(true);
      cameraSim.enableProcessedStream(true);
      sim.addCamera(cameraSim, AprilTagPoseEstimator.cameraTransforms.get(name));
    }
  }

  public static void updateSim() {
    sim.update(Robot.groundTruthPoseSupplier.get());
  }

  @Override
  public void updateInputs(AprilTagPoseEstimatorIOInputs inputs) {
    inputs.results = camera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);
  }
}
