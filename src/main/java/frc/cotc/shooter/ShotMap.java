// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Map;
import java.util.TreeMap;

public class ShotMap {
  private final TreeMap<Double, AngleEntry> map = new TreeMap<>();

  final void put(double distanceMeters, AngleEntry entry) {
    map.put(distanceMeters, entry);
  }

  public ShotResult get(double distanceMeters, double vxMetersPerSec, double vyMetersPerSec) {
    if (vyMetersPerSec < 0) {
      var flippedResult = get(distanceMeters, vxMetersPerSec, -vyMetersPerSec);
      return new ShotResult(
          flippedResult.pitchRad,
          flippedResult.yawRad.unaryMinus(),
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
      if (ShotResult.isInvalid(floor) && t >= 0.5) {
        return ShotResult.isInvalid(ceiling) ? null : ceiling;
      }
      if (ShotResult.isInvalid(ceiling) && t <= 0.5) {
        return ShotResult.isInvalid(floor) ? null : floor;
      }
      if (ShotResult.isInvalid(ceiling) || ShotResult.isInvalid(floor)) {
        return null;
      }
      return floor.interpolate(ceiling, t);
    }
    return val.get(angleRad, velMetersPerSec);
  }

  public record ShotResult(double pitchRad, Rotation2d yawRad, double velocityMetersPerSecond) {
    public ShotResult interpolate(ShotResult endValue, double t) {
      return new ShotResult(
          MathUtil.interpolate(pitchRad, endValue.pitchRad, t),
          yawRad.interpolate(endValue.yawRad, t),
          MathUtil.interpolate(velocityMetersPerSecond, endValue.velocityMetersPerSecond, t));
    }

    static boolean isInvalid(ShotResult result) {
      return result.pitchRad < 0;
    }

    static final ShotResult invalid = new ShotResult(-1, null, -1);
  }

  static class AngleEntry {
    private final TreeMap<Double, VelocityEntry> map = new TreeMap<>();

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
        if (ShotResult.isInvalid(floor) && t >= 0.5) {
          return ShotResult.isInvalid(ceiling) ? null : ceiling;
        }
        if (ShotResult.isInvalid(ceiling) && t <= 0.5) {
          return ShotResult.isInvalid(floor) ? null : floor;
        }
        if (ShotResult.isInvalid(ceiling) || ShotResult.isInvalid(floor)) {
          return null;
        }
        return floor.interpolate(ceiling, t);
      }
      return val.get(velocityMetersPerSecond);
    }
  }

  static class VelocityEntry {
    private final TreeMap<Double, ShotResult> map = new TreeMap<>();

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
        if (ShotResult.isInvalid(floor) && t >= 0.5) {
          return ShotResult.isInvalid(ceiling) ? null : ceiling;
        }
        if (ShotResult.isInvalid(ceiling) && t <= 0.5) {
          return ShotResult.isInvalid(floor) ? null : floor;
        }
        if (ShotResult.isInvalid(ceiling) || ShotResult.isInvalid(floor)) {
          return null;
        }
        return floor.interpolate(ceiling, t);
      }
      return val;
    }
  }
}
