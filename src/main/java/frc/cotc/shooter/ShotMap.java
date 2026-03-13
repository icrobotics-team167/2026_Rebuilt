// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.Map;

public class ShotMap extends InterpolatingTreeMap<Double, ShotMap.ShotResult> {
  public static ShotMap loadFromDeploy(String filePath) {
    try {
      System.out.println(
          "Loading shot map from " + Filesystem.getDeployDirectory() + File.separator + filePath);
      var mapper = new ObjectMapper();
      mapper.configure(JsonParser.Feature.INCLUDE_SOURCE_IN_LOCATION, true);
      var map =
          mapper.readValue(
              new File(Filesystem.getDeployDirectory() + File.separator + filePath), ShotMap.class);
      System.out.println("Loaded shot map");
      return map;
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public ShotMap(@JsonProperty("map") Map<Double, ShotResult> map) {
    super(MathUtil::inverseInterpolate, ShotResult::interpolate);
    map.forEach(this::put);
  }

  public record ShotResult(
      @JsonProperty("pitchRad") double pitchRad,
      @JsonProperty("speedMetersPerSec") double speedMetersPerSec,
      @JsonProperty("timeOfFlightSeconds") double timeOfFlightSeconds) {
    public ShotResult interpolate(ShotResult endValue, double t) {
      return new ShotResult(
          MathUtil.interpolate(pitchRad, endValue.pitchRad, t),
          MathUtil.interpolate(speedMetersPerSec, endValue.speedMetersPerSec, t),
          MathUtil.interpolate(timeOfFlightSeconds, endValue.timeOfFlightSeconds, t));
    }
  }

  public Double getTimeOfFlightDerivative(double distanceMeters) {
    var epsilon = 0.01;
    var tofPlusEpsilon = get(distanceMeters + epsilon).timeOfFlightSeconds();
    var tofMinusEpsilon = get(distanceMeters - epsilon).timeOfFlightSeconds();
    return (tofPlusEpsilon - tofMinusEpsilon) / (2 * epsilon);
  }
}
