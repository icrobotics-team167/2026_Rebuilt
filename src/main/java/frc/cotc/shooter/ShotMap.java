// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import java.util.Map;
import java.util.TreeMap;

public class ShotMap {
  private final TreeMap<Double, InterpolatingTreeMap<Double, ShotResult>> resultsMap =
      new TreeMap<>();
  private final InterpolatingDoubleTreeMap minimumVelsMap = new InterpolatingDoubleTreeMap();

  @SafeVarargs
  public final void put(
      double distanceMeters, Map.Entry<Double, ShotResult>... shotVelToPitchEntries) {
    minimumVelsMap.put(distanceMeters, shotVelToPitchEntries[0].getKey());
    var shotVelToResultsMap =
        new InterpolatingTreeMap<>(MathUtil::inverseInterpolate, ShotResult::interpolate);
    for (var entry : shotVelToPitchEntries) {
      shotVelToResultsMap.put(entry.getKey(), entry.getValue());
    }
    resultsMap.put(distanceMeters, shotVelToResultsMap);
  }

  public record ShotResult(double pitchRad, double timeToTargetSeconds) {
    public ShotResult interpolate(ShotResult endValue, double t) {
      return new ShotResult(
          MathUtil.interpolate(pitchRad, endValue.pitchRad, t),
          MathUtil.interpolate(timeToTargetSeconds, endValue.timeToTargetSeconds, t));
    }
  }

  public ShotResult get(double distanceMeters, double velocityMetersPerSecond) {
    var val = resultsMap.get(distanceMeters);
    if (val == null) {
      var ceilingKey = resultsMap.ceilingKey(distanceMeters);
      var floorKey = resultsMap.floorKey(distanceMeters);

      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return resultsMap.get(floorKey).get(velocityMetersPerSecond);
      }
      if (floorKey == null) {
        return resultsMap.get(ceilingKey).get(velocityMetersPerSecond);
      }
      var floor = resultsMap.get(floorKey);
      var ceiling = resultsMap.get(ceilingKey);
      return floor
          .get(velocityMetersPerSecond)
          .interpolate(
              ceiling.get(velocityMetersPerSecond),
              MathUtil.inverseInterpolate(floorKey, ceilingKey, distanceMeters));
    } else {
      return val.get(velocityMetersPerSecond);
    }
  }

  public boolean isPossible(double distanceMeters, double velocityMetersPerSecond) {
    return minimumVelsMap.get(distanceMeters) < velocityMetersPerSecond;
  }
}
