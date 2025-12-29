// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.*;

public interface AprilTagPoseEstimatorIO {
  class AprilTagPoseEstimatorIOInputs implements LoggableInputs {
    PhotonPipelineResult[] results = new PhotonPipelineResult[0];

    @Override
    public void toLog(LogTable table) {
      table.put("results", results.length);
      for (int i = 0; i < results.length; i++) {
        logResult(table.getSubtable("result_" + i), results[i]);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      results = new PhotonPipelineResult[table.get("results", 0)];
      for (int i = 0; i < results.length; i++) {
        results[i] = getResult(table.getSubtable("result_" + i));
      }
    }

    private void logResult(LogTable table, PhotonPipelineResult result) {
      table.put("metadata", PhotonStructs.metadataStruct, result.metadata);
      table.put(
          "targets",
          PhotonStructs.trackedTargetStruct,
          result.targets.toArray(new PhotonTrackedTarget[0]));
      table.put("hasMultitagResult", result.multitagResult.isPresent());
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();
        table.put(
            "multitagResult/estimatedPose",
            PhotonStructs.pnpResultStruct,
            multitagResult.estimatedPose);
        // Can't directly log List<Short> so convert to int[]
        var fiducialIDsUsed = new int[multitagResult.fiducialIDsUsed.size()];
        for (int i = 0; i < fiducialIDsUsed.length; i++) {
          fiducialIDsUsed[i] = multitagResult.fiducialIDsUsed.get(i);
        }
        table.put("multitagResult/fiducialIDsUsed", fiducialIDsUsed);
      }
    }

    private PhotonPipelineResult getResult(LogTable table) {
      var metadata =
          table.get("metadata", PhotonStructs.metadataStruct, new PhotonPipelineMetadata());
      var targets = table.get("targets", PhotonStructs.trackedTargetStruct);
      var hasMultitagResult = table.get("hasMultitagResult", false);
      if (hasMultitagResult) {
        var estimatedPose =
            table.get(
                "multitagResult/estimatedPose", PhotonStructs.pnpResultStruct, new PnpResult());
        var fiducialIDsUsedInts = table.get("multitagResult/fiducialIDsUsed", new int[0]);
        var fiducialIDsUsed = new ArrayList<Short>();
        for (var id : fiducialIDsUsedInts) {
          fiducialIDsUsed.add((short) id);
        }
        var multitagResult = new MultiTargetPNPResult(estimatedPose, fiducialIDsUsed);
        return new PhotonPipelineResult(metadata, List.of(targets), Optional.of(multitagResult));
      } else {
        return new PhotonPipelineResult(metadata, List.of(targets), Optional.empty());
      }
    }
  }

  default void updateInputs(AprilTagPoseEstimatorIOInputs inputs) {}
}
