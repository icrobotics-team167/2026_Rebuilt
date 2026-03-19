// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Shifts {
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  private static final Timer shiftTimer = new Timer();
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  public static final double autoEndTime = 20.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

  private static final LoggedNetworkBoolean allianceWinOverride =
      new LoggedNetworkBoolean("AllianceWinOverride");

  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Return override value
    if (allianceWinOverride.get()) {
      return alliance;
    }

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (!message.isEmpty()) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // Return default value
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the begining of teleop. */
  public static void initialize() {
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double currentTime = shiftTimer.get();
    double stateTimeElapsed = shiftTimer.get();
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }
    ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    return shiftInfo;
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  private static final double minFuelCountDelay = 1.0;
  private static final double maxFuelCountDelay = 2.0;
  private static final double shiftEndFuelCountExtension = 3.0;

  public static ShiftInfo getAdjustedShiftInfo(double timeOfFlight) {
    double approachingActiveFudge = -1 * (timeOfFlight + minFuelCountDelay);
    double endingActiveFudge = shiftEndFuelCountExtension + -1 * (timeOfFlight + maxFuelCountDelay);

    boolean[] shiftSchedule = getSchedule();
    // Starting active
    if (shiftSchedule[1]) {
      double[] shiftedShiftStartTimes = {
        0.0,
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge
      };
      double[] shiftedShiftEndTimes = {
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge,
        140.0
      };
      return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }
    double[] shiftedShiftStartTimes = {
      0.0,
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0
    };
    double[] shiftedShiftEndTimes = {
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0,
      140.0
    };
    return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
  }
}
