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
      @JsonProperty("pitchRad") double pitchRad, @JsonProperty("yaw") Rotation2d yaw) {
    public ShotResult interpolate(ShotResult endValue, double t) {
      return new ShotResult(
          MathUtil.interpolate(pitchRad, endValue.pitchRad, t), yaw.interpolate(endValue.yaw, t));
    }

    static boolean isInvalid(ShotResult result) {
      return result.pitchRad < 0;
    }

    static ShotResult nullIfInvalid(ShotResult result) {
      return isInvalid(result) ? null : result;
    }
  }

  public ShotResult get(
      double distanceMeters,
      double vxMetersPerSec,
      double vyMetersPerSec,
      double shotSpeedMetersPerSec) {
    if (vyMetersPerSec < 0) {
      var flippedResult =
          get(distanceMeters, vxMetersPerSec, -vyMetersPerSec, shotSpeedMetersPerSec);
      if (flippedResult == null) {
        return null;
      }
      return new ShotResult(flippedResult.pitchRad, flippedResult.yaw.unaryMinus());
    }
    var robotSpeedMetersPerSec =
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
        return map.get(floorKey).get(angleRad, robotSpeedMetersPerSec, shotSpeedMetersPerSec);
      }
      if (floorKey == null) {
        return map.get(ceilingKey).get(angleRad, robotSpeedMetersPerSec, shotSpeedMetersPerSec);
      }
      var floor = map.get(floorKey).get(angleRad, robotSpeedMetersPerSec, shotSpeedMetersPerSec);
      var ceiling =
          map.get(ceilingKey).get(angleRad, robotSpeedMetersPerSec, shotSpeedMetersPerSec);
      var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, distanceMeters);
      if (floor == null || ceiling == null) {
        return null;
      }
      return floor.interpolate(ceiling, t);
    }
    return val.get(angleRad, robotSpeedMetersPerSec, shotSpeedMetersPerSec);
  }

  public Double getMinSpeed(double distanceMeters, double vxMetersPerSec, double vyMetersPerSec) {
    if (vyMetersPerSec < 0) {
      return getMinSpeed(distanceMeters, vxMetersPerSec, -vyMetersPerSec);
    }
    var robotSpeedMetersPerSec =
        Math.sqrt(vxMetersPerSec * vxMetersPerSec + vyMetersPerSec * vyMetersPerSec);
    var robotVelocityAngleRad = Math.atan2(vyMetersPerSec, vxMetersPerSec);

    var val = map.get(distanceMeters);
    if (val == null) {
      var ceilingKey = map.ceilingKey(distanceMeters);
      var floorKey = map.floorKey(distanceMeters);
      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return map.get(floorKey).getMinSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
      }
      if (floorKey == null) {
        return map.get(ceilingKey).getMinSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
      }
      var floor = map.get(floorKey).getMinSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
      var ceiling = map.get(ceilingKey).getMinSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
      var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, distanceMeters);
      if (floor == null && t > 0.5) {
        return ceiling;
      }
      if (ceiling == null && t < 0.5) {
        return floor;
      }
      if (floor == null || ceiling == null) {
        return null;
      }
      return MathUtil.interpolate(floor, ceiling, t);
    }
    return val.getMinSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
  }

  public Double getMaxSpeed(double distanceMeters, double vxMetersPerSec, double vyMetersPerSec) {
    if (vyMetersPerSec < 0) {
      return getMaxSpeed(distanceMeters, vxMetersPerSec, -vyMetersPerSec);
    }
    var robotSpeedMetersPerSec =
        Math.sqrt(vxMetersPerSec * vxMetersPerSec + vyMetersPerSec * vyMetersPerSec);
    var robotVelocityAngleRad = Math.atan2(vyMetersPerSec, vxMetersPerSec);
    var val = map.get(distanceMeters);
    if (val == null) {
      var ceilingKey = map.ceilingKey(distanceMeters);
      var floorKey = map.floorKey(distanceMeters);
      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return map.get(floorKey).getMaxSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
      }
      if (floorKey == null) {
        return map.get(ceilingKey).getMaxSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
      }
      var floor = map.get(floorKey).getMaxSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
      var ceiling = map.get(ceilingKey).getMaxSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
      var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, distanceMeters);
      if (floor == null && t > 0.5) {
        return ceiling;
      }
      if (ceiling == null && t < 0.5) {
        return floor;
      }
      if (floor == null || ceiling == null) {
        return null;
      }
      return MathUtil.interpolate(floor, ceiling, t);
    }
    return val.getMaxSpeed(robotVelocityAngleRad, robotSpeedMetersPerSec);
  }

  static class AngleEntry {
    @JsonProperty("map")
    private final TreeMap<Double, RobotSpeedEntry> map = new TreeMap<>();

    public AngleEntry() {}

    @SafeVarargs
    AngleEntry(Map.Entry<Double, RobotSpeedEntry>... entries) {
      for (var entry : entries) {
        map.put(entry.getKey(), entry.getValue());
      }
    }

    ShotResult get(
        double robotVelocityAngleRad, double robotSpeedMetersPerSec, double shotSpeedMetersPerSec) {
      var val = map.get(robotVelocityAngleRad);
      if (val == null) {
        var ceilingKey = map.ceilingKey(robotVelocityAngleRad);
        var floorKey = map.floorKey(robotVelocityAngleRad);
        if (ceilingKey == null && floorKey == null) {
          return null;
        }
        if (ceilingKey == null) {
          return map.get(floorKey).get(robotSpeedMetersPerSec, shotSpeedMetersPerSec);
        }
        if (floorKey == null) {
          return map.get(ceilingKey).get(robotSpeedMetersPerSec, shotSpeedMetersPerSec);
        }
        var floor = map.get(floorKey).get(robotSpeedMetersPerSec, shotSpeedMetersPerSec);
        var ceiling = map.get(ceilingKey).get(robotSpeedMetersPerSec, shotSpeedMetersPerSec);
        var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, robotVelocityAngleRad);
        if (floor == null && t > 0.5) {
          return ceiling;
        }
        if (ceiling == null && t < 0.5) {
          return floor;
        }
        if (floor == null || ceiling == null) {
          return null;
        }
        return floor.interpolate(ceiling, t);
      }
      return val.get(robotSpeedMetersPerSec, shotSpeedMetersPerSec);
    }

    Double getMinSpeed(double robotVelocityAngleRad, double robotSpeedMetersPerSec) {
      var val = map.get(robotVelocityAngleRad);
      if (val == null) {
        var ceilingKey = map.ceilingKey(robotVelocityAngleRad);
        var floorKey = map.floorKey(robotVelocityAngleRad);
        if (ceilingKey == null && floorKey == null) {
          return null;
        }
        if (ceilingKey == null) {
          return map.get(floorKey).getMinSpeed(robotSpeedMetersPerSec);
        }
        if (floorKey == null) {
          return map.get(ceilingKey).getMinSpeed(robotSpeedMetersPerSec);
        }
        var floor = map.get(floorKey).getMinSpeed(robotSpeedMetersPerSec);
        var ceiling = map.get(ceilingKey).getMinSpeed(robotSpeedMetersPerSec);
        var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, robotVelocityAngleRad);
        if (floor == null && t > 0.5) {
          return ceiling;
        }
        if (ceiling == null && t < 0.5) {
          return floor;
        }
        if (floor == null || ceiling == null) {
          return null;
        }
        return MathUtil.interpolate(floor, ceiling, t);
      }
      return val.getMinSpeed(robotSpeedMetersPerSec);
    }

    Double getMaxSpeed(double robotVelocityAngleRad, double robotSpeedMetersPerSec) {
      var val = map.get(robotVelocityAngleRad);
      if (val == null) {
        var ceilingKey = map.ceilingKey(robotVelocityAngleRad);
        var floorKey = map.floorKey(robotVelocityAngleRad);
        if (ceilingKey == null && floorKey == null) {
          return null;
        }
        if (ceilingKey == null) {
          return map.get(floorKey).getMaxSpeed(robotSpeedMetersPerSec);
        }
        if (floorKey == null) {
          return map.get(ceilingKey).getMaxSpeed(robotSpeedMetersPerSec);
        }
        var floor = map.get(floorKey).getMaxSpeed(robotSpeedMetersPerSec);
        var ceiling = map.get(ceilingKey).getMaxSpeed(robotSpeedMetersPerSec);
        var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, robotVelocityAngleRad);
        if (floor == null && t > 0.5) {
          return ceiling;
        }
        if (ceiling == null && t < 0.5) {
          return floor;
        }
        if (floor == null || ceiling == null) {
          return null;
        }
        return MathUtil.interpolate(floor, ceiling, t);
      }
      return val.getMaxSpeed(robotSpeedMetersPerSec);
    }
  }

  static class RobotSpeedEntry {
    private final TreeMap<Double, ShotSpeedEntry> map;

    final TreeMap<Double, Double> minSpeedMap = new TreeMap<>();
    final TreeMap<Double, Double> maxSpeedMap = new TreeMap<>();

    RobotSpeedEntry(@JsonProperty("map") TreeMap<Double, ShotSpeedEntry> map) {
      this.map = map;
      for (var entry : map.entrySet()) {
        minSpeedMap.put(entry.getKey(), entry.getValue().minSpeedMetersPerSecond);
        maxSpeedMap.put(entry.getKey(), entry.getValue().maxSpeedMetersPerSecond);
      }
    }

    ShotResult get(double robotSpeedMetersPerSec, double shotSpeedMetersPerSec) {
      var val = map.get(robotSpeedMetersPerSec);
      if (val == null) {
        var ceilingKey = map.ceilingKey(robotSpeedMetersPerSec);
        var floorKey = map.floorKey(robotSpeedMetersPerSec);
        if (ceilingKey == null && floorKey == null) {
          return null;
        }
        if (ceilingKey == null) {
          return map.get(floorKey).get(shotSpeedMetersPerSec);
        }
        if (floorKey == null) {
          return map.get(ceilingKey).get(shotSpeedMetersPerSec);
        }
        var floor = map.get(floorKey).get(shotSpeedMetersPerSec);
        var ceiling = map.get(ceilingKey).get(shotSpeedMetersPerSec);
        var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, robotSpeedMetersPerSec);
        if (floor == null && t > 0.5) {
          return ceiling;
        }
        if (ceiling == null && t < 0.5) {
          return floor;
        }
        if (floor == null || ceiling == null) {
          return null;
        }
        return floor.interpolate(ceiling, t);
      }
      return val.get(shotSpeedMetersPerSec);
    }

    Double getMinSpeed(double robotSpeedMetersPerSec) {
      var val = minSpeedMap.get(robotSpeedMetersPerSec);
      if (val == null) {
        var ceilingKey = minSpeedMap.ceilingKey(robotSpeedMetersPerSec);
        var floorKey = minSpeedMap.floorKey(robotSpeedMetersPerSec);
        if (ceilingKey == null && floorKey == null) {
          return null;
        }
        if (ceilingKey == null) {
          var floor = minSpeedMap.get(floorKey);
          if (floor == null) {
            return null;
          }
          return floor;
        }
        if (floorKey == null) {
          var ceiling = minSpeedMap.get(ceilingKey);
          if (ceiling == null) {
            return null;
          }
          return ceiling;
        }
        var floor = minSpeedMap.get(floorKey);
        var ceiling = minSpeedMap.get(ceilingKey);
        var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, robotSpeedMetersPerSec);
        if (floor == null && t > 0.5) {
          return ceiling;
        }
        if (ceiling == null && t < 0.5) {
          return floor;
        }
        if (floor == null || ceiling == null) {
          return null;
        }
        return MathUtil.interpolate(floor, ceiling, t);
      }
      return val;
    }

    Double getMaxSpeed(double robotSpeedMetersPerSec) {
      var val = maxSpeedMap.get(robotSpeedMetersPerSec);
      if (val == null) {
        var ceilingKey = maxSpeedMap.ceilingKey(robotSpeedMetersPerSec);
        var floorKey = maxSpeedMap.floorKey(robotSpeedMetersPerSec);
        if (ceilingKey == null && floorKey == null) {
          return null;
        }
        if (ceilingKey == null) {
          var floor = maxSpeedMap.get(floorKey);
          if (floor == null) {
            return null;
          }
          if (floor < 0) {
            return null;
          }
          return floor;
        }
        if (floorKey == null) {
          var ceiling = maxSpeedMap.get(ceilingKey);
          if (ceiling == null) {
            return null;
          }
          if (ceiling < 0) {
            return null;
          }
          return ceiling;
        }
        var floor = maxSpeedMap.get(floorKey);
        var ceiling = maxSpeedMap.get(ceilingKey);
        var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, robotSpeedMetersPerSec);
        if (floor == null && t > 0.5) {
          return ceiling;
        }
        if (ceiling == null && t < 0.5) {
          return floor;
        }
        if (floor == null || ceiling == null) {
          return null;
        }
        return MathUtil.interpolate(floor, ceiling, t);
      }
      return val;
    }
  }

  static class ShotSpeedEntry {
    private final TreeMap<Double, ShotResult> map;

    final Double minSpeedMetersPerSecond;
    final Double maxSpeedMetersPerSecond;

    ShotSpeedEntry(@JsonProperty("map") TreeMap<Double, ShotResult> map) {
      this.map = map;
      var keys = map.keySet().toArray(new Double[0]);
      if (keys.length < 3) {
        minSpeedMetersPerSecond = null;
        maxSpeedMetersPerSecond = null;
      } else {
        minSpeedMetersPerSecond = keys[1];
        maxSpeedMetersPerSecond = keys[keys.length - 2];
      }
    }

    ShotResult get(double shotSpeedMetersPerSec) {
      var val = map.get(shotSpeedMetersPerSec);
      if (val == null) {
        var ceilingKey = map.ceilingKey(shotSpeedMetersPerSec);
        var floorKey = map.floorKey(shotSpeedMetersPerSec);
        if (ceilingKey == null && floorKey == null) {
          return null;
        }
        if (ceilingKey == null) {
          return ShotResult.nullIfInvalid(map.get(floorKey));
        }
        if (floorKey == null) {
          return ShotResult.nullIfInvalid(map.get(ceilingKey));
        }
        var floor = map.get(floorKey);
        var ceiling = map.get(ceilingKey);
        var t = MathUtil.inverseInterpolate(floorKey, ceilingKey, shotSpeedMetersPerSec);
        if (ShotResult.isInvalid(floor) && t > 0.5) {
          return ShotResult.nullIfInvalid(ceiling);
        }
        if (ShotResult.isInvalid(ceiling) && t < 0.5) {
          return ShotResult.nullIfInvalid(floor);
        }
        if (ShotResult.isInvalid(floor) || ShotResult.isInvalid(ceiling)) {
          return null;
        }
        return ShotResult.nullIfInvalid(floor.interpolate(ceiling, t));
      }
      return ShotResult.nullIfInvalid(val);
    }
  }
}
