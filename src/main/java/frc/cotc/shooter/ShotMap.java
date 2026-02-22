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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.Map;
import java.util.TreeMap;

public class ShotMap {
  private final TreeMap<Double, ShotSpeedEntry> map;
  private final InterpolatingDoubleTreeMap minSpeedsMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap maxSpeedsMap = new InterpolatingDoubleTreeMap();

  public static ShotMap loadFromDeploy(String filePath) {
    try {
      System.out.println(
          "Loading shot map from " + Filesystem.getDeployDirectory() + File.separator + filePath);
      var mapper = new ObjectMapper();
      mapper.configure(JsonParser.Feature.INCLUDE_SOURCE_IN_LOCATION, true);
      return mapper.readValue(
          new File(Filesystem.getDeployDirectory() + File.separator + filePath), ShotMap.class);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public record ShotResult(
      @JsonProperty("pitchRad") double pitchRad,
      @JsonProperty("timeOfFlightSeconds") double timeOfFlightSeconds) {
    public ShotResult interpolate(ShotResult endValue, double t) {
      return new ShotResult(
          MathUtil.interpolate(pitchRad, endValue.pitchRad, t),
          MathUtil.interpolate(timeOfFlightSeconds, endValue.timeOfFlightSeconds, t));
    }
  }

  ShotMap(@JsonProperty("map") TreeMap<Double, ShotSpeedEntry> map) {
    this.map = map;
    for (var entry : map.entrySet()) {
      var distance = entry.getKey();
      var value = entry.getValue();
      minSpeedsMap.put(distance, value.minSpeedMetersPerSecond);
      maxSpeedsMap.put(distance, value.maxSpeedMetersPerSecond);
    }
  }

  public ShotResult get(double distanceMeters, double shotSpeedMetersPerSecond) {
    var val = map.get(distanceMeters);
    if (val == null) {
      var ceilingKey = map.ceilingKey(distanceMeters);
      var floorKey = map.floorKey(distanceMeters);

      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return map.get(floorKey).get(shotSpeedMetersPerSecond);
      }
      if (floorKey == null) {
        return map.get(ceilingKey).get(shotSpeedMetersPerSecond);
      }
      var floor = map.get(floorKey).get(shotSpeedMetersPerSecond);
      var ceiling = map.get(ceilingKey).get(shotSpeedMetersPerSecond);

      return floor.interpolate(
          ceiling, MathUtil.inverseInterpolate(distanceMeters, floorKey, ceilingKey));
    } else {
      return val.get(shotSpeedMetersPerSecond);
    }
  }

  public Double getTimeOfFlightDerivative(double distanceMeters, double shotSpeedMetersPerSecond) {
    var val = map.get(distanceMeters);
    if (val == null) {
      var ceilingKey = map.ceilingKey(distanceMeters);
      var floorKey = map.floorKey(distanceMeters);

      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return map.get(floorKey).getTimeOfFlightDerivative(shotSpeedMetersPerSecond);
      }
      if (floorKey == null) {
        return map.get(ceilingKey).getTimeOfFlightDerivative(shotSpeedMetersPerSecond);
      }
      var floor = map.get(floorKey).getTimeOfFlightDerivative(shotSpeedMetersPerSecond);
      var ceiling = map.get(ceilingKey).getTimeOfFlightDerivative(shotSpeedMetersPerSecond);

      return MathUtil.interpolate(
          floor, ceiling, MathUtil.inverseInterpolate(distanceMeters, floorKey, ceilingKey));
    } else {
      return val.getTimeOfFlightDerivative(shotSpeedMetersPerSecond);
    }
  }

  public double getMinSpeed(double distanceMeters) {
    return minSpeedsMap.get(distanceMeters);
  }

  public double getMaxSpeed(double distanceMeters) {
    return maxSpeedsMap.get(distanceMeters);
  }

  static class ShotSpeedEntry {
    private final InterpolatingTreeMap<Double, ShotResult> shotSpeedToResultMap =
        new InterpolatingTreeMap<>(MathUtil::inverseInterpolate, ShotResult::interpolate);
    private final InterpolatingDoubleTreeMap shotSpeedToTimeOfFlightDerivativeMap =
        new InterpolatingDoubleTreeMap();
    final double minSpeedMetersPerSecond;
    final double maxSpeedMetersPerSecond;

    ShotSpeedEntry(@JsonProperty("map") Map<Double, ShotResult> map) {
      for (var entry : map.entrySet()) {
        this.shotSpeedToResultMap.put(entry.getKey(), entry.getValue());
      }
      var speedsArray = map.keySet().toArray(new Double[0]);
      minSpeedMetersPerSecond = speedsArray[0];
      maxSpeedMetersPerSecond = speedsArray[speedsArray.length - 1];
      var valuesArray = map.values().toArray(new ShotResult[0]);
      if (valuesArray.length > 1) {
        // Slope from first entry to second entry
        shotSpeedToTimeOfFlightDerivativeMap.put(
            speedsArray[0],
            (valuesArray[1].timeOfFlightSeconds - valuesArray[0].timeOfFlightSeconds)
                / (speedsArray[1] - speedsArray[0]));
        for (int i = 1; i < speedsArray.length - 2; i++) {
          // Average of slopes between this point and the two adjacent points
          var lowerSlope =
              (valuesArray[i].timeOfFlightSeconds - valuesArray[i - 1].timeOfFlightSeconds)
                  / (speedsArray[i] - speedsArray[i - 1]);
          var upperSlope =
              (valuesArray[i + 1].timeOfFlightSeconds - valuesArray[i].timeOfFlightSeconds)
                  / (speedsArray[i + 1] - speedsArray[i]);
          shotSpeedToTimeOfFlightDerivativeMap.put(speedsArray[i], (upperSlope + lowerSlope) / 2.0);
        }
        // Slope from last entry to second to last entry
        shotSpeedToTimeOfFlightDerivativeMap.put(
            speedsArray[speedsArray.length - 1],
            (valuesArray[valuesArray.length - 1].timeOfFlightSeconds
                    - valuesArray[valuesArray.length - 2].timeOfFlightSeconds)
                / (speedsArray[speedsArray.length - 1] - speedsArray[speedsArray.length - 2]));
      } else {
        shotSpeedToTimeOfFlightDerivativeMap.put(speedsArray[0], 0.0);
      }
    }

    ShotResult get(double shotSpeedMetersPerSec) {
      return shotSpeedToResultMap.get(shotSpeedMetersPerSec);
    }

    double getTimeOfFlightDerivative(double shotSpeedMetersPerSec) {
      return shotSpeedToTimeOfFlightDerivativeMap.get(shotSpeedMetersPerSec);
    }
  }
}
