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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.Map;
import java.util.TreeMap;

public class ShotMap {
  @JsonProperty("map")
  private final TreeMap<Double, AngleEntry> map = new TreeMap<>();

  public static ShotMap loadFromDeploy(String filePath) {
    try {
      var mapper = new ObjectMapper();
      mapper.configure(JsonParser.Feature.INCLUDE_SOURCE_IN_LOCATION, true);
      return mapper.readValue(
          new File(Filesystem.getDeployDirectory() + File.separator + filePath), ShotMap.class);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  final void put(double distanceMeters, AngleEntry entry) {
    map.put(distanceMeters, entry);
  }

  public ShotResult get(double distanceMeters, double vxMetersPerSec, double vyMetersPerSec) {
    if (vyMetersPerSec < 0) {
      var flippedResult = get(distanceMeters, vxMetersPerSec, -vyMetersPerSec);
      if (flippedResult == null) {
        return null;
      }
      return new ShotResult(
          flippedResult.pitchRad,
          flippedResult.yaw.unaryMinus(),
          flippedResult.velocityMetersPerSecond);
    }
    var velMetersPerSec =
        Math.sqrt(vxMetersPerSec * vxMetersPerSec + vyMetersPerSec * vyMetersPerSec);
    var angleRad = Math.atan2(vyMetersPerSec, vxMetersPerSec);

    var val = map.get(distanceMeters);
    if (val == null) {
      var ceilingKey = map.ceilingKey(distanceMeters);
      var floorKey = map.floorKey(distanceMeters);
      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return map.get(floorKey).get(angleRad, velMetersPerSec);
      }
      if (floorKey == null) {
        return map.get(ceilingKey).get(angleRad, velMetersPerSec);
      }
      var floor = map.get(floorKey).get(angleRad, velMetersPerSec);
      var ceiling = map.get(ceilingKey).get(angleRad, velMetersPerSec);
      var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, distanceMeters);
      if (floor == null || ceiling == null) {
        return null;
      }
      if (ShotResult.isInvalid(floor) || ShotResult.isInvalid(ceiling)) {
        return null;
      }
      return floor.interpolate(ceiling, t);
    }
    return val.get(angleRad, velMetersPerSec);
  }

  public record ShotResult(
      @JsonProperty("pitchRad") double pitchRad,
      @JsonProperty("yaw") Rotation2d yaw,
      @JsonProperty("velocityMetersPerSecond") double velocityMetersPerSecond) {
    public ShotResult interpolate(ShotResult endValue, double t) {
      return new ShotResult(
          MathUtil.interpolate(pitchRad, endValue.pitchRad, t),
          yaw.interpolate(endValue.yaw, t),
          MathUtil.interpolate(velocityMetersPerSecond, endValue.velocityMetersPerSecond, t));
    }

    static boolean isInvalid(ShotResult result) {
      return result.pitchRad < 0;
    }

    static final ShotResult invalid = new ShotResult(-1, null, -1);
  }

  static class AngleEntry {
    @JsonProperty("map")
    private final TreeMap<Double, VelocityEntry> map = new TreeMap<>();

    public AngleEntry() {}

    @SafeVarargs
    AngleEntry(Map.Entry<Double, VelocityEntry>... entries) {
      for (var entry : entries) {
        map.put(entry.getKey(), entry.getValue());
      }
    }

    ShotResult get(double angleRad, double velocityMetersPerSecond) {
      var val = map.get(angleRad);
      if (val == null) {
        var ceilingKey = map.ceilingKey(angleRad);
        var floorKey = map.floorKey(angleRad);
        if (ceilingKey == null && floorKey == null) {
          return null;
        }
        if (ceilingKey == null) {
          return map.get(floorKey).get(velocityMetersPerSecond);
        }
        if (floorKey == null) {
          return map.get(ceilingKey).get(velocityMetersPerSecond);
        }
        var floor = map.get(floorKey).get(velocityMetersPerSecond);
        var ceiling = map.get(ceilingKey).get(velocityMetersPerSecond);
        var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, angleRad);
        if (floor == null || ceiling == null) {
          return null;
        }
        if (ShotResult.isInvalid(floor) || ShotResult.isInvalid(ceiling)) {
          return null;
        }
        return floor.interpolate(ceiling, t);
      }
      return val.get(velocityMetersPerSecond);
    }
  }

  static class VelocityEntry {
    @JsonProperty("map")
    private final TreeMap<Double, ShotResult> map = new TreeMap<>();

    public VelocityEntry() {}

    @SafeVarargs
    public VelocityEntry(Map.Entry<Double, ShotResult>... entries) {
      for (var entry : entries) {
        map.put(entry.getKey(), entry.getValue());
      }
    }

    ShotResult get(double velocityMetersPerSecond) {
      var val = map.get(velocityMetersPerSecond);
      if (val == null) {
        var ceilingKey = map.ceilingKey(velocityMetersPerSecond);
        var floorKey = map.floorKey(velocityMetersPerSecond);
        if (ceilingKey == null && floorKey == null) {
          return null;
        }
        if (ceilingKey == null) {
          return map.get(floorKey);
        }
        if (floorKey == null) {
          return map.get(ceilingKey);
        }
        var floor = map.get(floorKey);
        var ceiling = map.get(ceilingKey);
        var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, velocityMetersPerSecond);
        if (ShotResult.isInvalid(floor) || ShotResult.isInvalid(ceiling)) {
          return null;
        }
        return floor.interpolate(ceiling, t);
      }
      return val;
    }
  }
}
