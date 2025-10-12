// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import edu.wpi.first.wpilibj2.command.*;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class CommandsLogging {
  private static final Set<Command> runningNonInterrupters = new HashSet<>();
  public static final Map<Command, Command> runningInterrupters = new HashMap<>();
  private static final Map<Subsystem, Command> requiredSubsystems = new HashMap<>();

  public static void commandStarted(final Command command) {
    if (!runningInterrupters.containsKey(command)) {
      runningNonInterrupters.add(command);
    }

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.put(subsystem, command);
    }
  }

  public static void commandEnded(final Command command) {
    runningNonInterrupters.remove(command);
    runningInterrupters.remove(command);

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.remove(subsystem);
    }
  }

  public static String getCommandName(Command command) {
    final StringBuilder subsystemsBuilder = new StringBuilder();
    final StringBuilder subCommandsNameBuilder = new StringBuilder();

    int i = 1;
    for (final var subsystem : command.getRequirements()) {
      subsystemsBuilder.append(subsystem.getName());
      if (i < command.getRequirements().size()) {
        subsystemsBuilder.append(", ");
      }

      i++;
    }

    String finalName;
    if (command instanceof SequentialCommandGroup sequence) {
      try {
        if (sequence.getName().equals("SequentialCommandGroup")) {
          var subCommandsField = sequence.getClass().getDeclaredField("m_commands");
          subCommandsField.setAccessible(true);
          //noinspection unchecked
          var subCommands = (List<Command>) subCommandsField.get(sequence);

          subCommandsNameBuilder.append("sequence(");
          int j = 1;
          for (final var subCommand : subCommands) {
            subCommandsNameBuilder.append(getCommandName(subCommand));
            if (j < subCommands.size()) {
              subCommandsNameBuilder.append(", ");
            }

            j++;
          }
          subCommandsNameBuilder.append(")");
          finalName = subCommandsNameBuilder.toString();
        } else {
          finalName = sequence.getName();
        }
      } catch (NoSuchFieldException | IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ParallelCommandGroup parallel) {
      try {
        if (parallel.getName().equals("ParallelCommandGroup")) {
          var subCommandsField = parallel.getClass().getDeclaredField("m_commands");
          subCommandsField.setAccessible(true);
          //noinspection unchecked
          var subCommands = (Map<Command, Boolean>) subCommandsField.get(parallel);

          subCommandsNameBuilder.append("parallel(");
          int j = 1;
          for (final var subCommand : subCommands.keySet()) {
            subCommandsNameBuilder.append(getCommandName(subCommand));
            if (j < subCommands.size()) {
              subCommandsNameBuilder.append(", ");
            }

            j++;
          }
          subCommandsNameBuilder.append(")");
          finalName = subCommandsNameBuilder.toString();
        } else {
          finalName = parallel.getName();
        }
      } catch (NoSuchFieldException | IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ParallelDeadlineGroup deadline) {
      try {
        if (deadline.getName().equals("DeadlineCommandGroup")) {
          var subCommandsField = deadline.getClass().getDeclaredField("m_commands");
          subCommandsField.setAccessible(true);
          //noinspection unchecked
          var subCommands = ((Map<Command, Boolean>) subCommandsField.get(deadline)).keySet();

          var deadlineCommandField = deadline.getClass().getDeclaredField("m_deadline");
          deadlineCommandField.setAccessible(true);
          subCommandsNameBuilder.append(
              getCommandName((Command) deadlineCommandField.get(deadline)));

          subCommandsNameBuilder.append(".deadlineFor(");
          int j = 1;
          for (final var subCommand : subCommands) {
            subCommandsNameBuilder.append(getCommandName(subCommand));
            if (j < subCommands.size()) {
              subCommandsNameBuilder.append(", ");
            }

            j++;
          }
          subCommandsNameBuilder.append(")");
          finalName = subCommandsNameBuilder.toString();
        } else {
          finalName = deadline.getName();
        }
      } catch (NoSuchFieldException | IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ParallelRaceGroup race) {
      try {
        if (race.getName().equals("ParallelRaceGroup")) {
          var subCommandsField = race.getClass().getDeclaredField("m_commands");
          subCommandsField.setAccessible(true);
          //noinspection unchecked
          var subCommands = (Set<Command>) subCommandsField.get(race);

          subCommandsNameBuilder.append("race(");
          int j = 1;
          for (final var subCommand : subCommands) {
            subCommandsNameBuilder.append(getCommandName(subCommand));
            if (j < subCommands.size()) {
              subCommandsNameBuilder.append(", ");
            }

            j++;
          }
          subCommandsNameBuilder.append(")");
          finalName = subCommandsNameBuilder.toString();
        } else {
          finalName = race.getName();
        }
      } catch (NoSuchFieldException | IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else {
      finalName = command.getName();
    }

    if (i > 1) {
      finalName += " (" + subsystemsBuilder + ")";
    }
    return finalName;
  }

  public static void logRunningCommands() {
    Logger.recordOutput("CommandScheduler/Running/.type", "Alerts");

    //    final String[] runningCommands = new String[runningNonInterrupters.size()];
    //    int i = 0;
    //    for (final Command command : runningNonInterrupters) {
    //      runningCommands[i] = getCommandName(command);
    //      i++;
    //    }
    final ArrayList<String> runningCommands = new ArrayList<>();
    final ArrayList<String> runningDefaultCommands = new ArrayList<>();
    for (final Command command : runningNonInterrupters) {
      boolean isDefaultCommand = false;
      for (Subsystem subsystem : command.getRequirements()) {
        if (subsystem.getDefaultCommand() == command) {
          runningDefaultCommands.add(getCommandName(command));
          isDefaultCommand = true;
          break;
        }
      }
      if (!isDefaultCommand) {
        runningCommands.add(getCommandName(command));
      }
    }
    Logger.recordOutput(
        "CommandScheduler/Running/warnings", runningCommands.toArray(new String[0]));
    Logger.recordOutput(
        "CommandScheduler/Running/infos", runningDefaultCommands.toArray(new String[0]));

    final String[] interrupters = new String[runningInterrupters.size()];
    int j = 0;
    for (final Map.Entry<Command, Command> entry : runningInterrupters.entrySet()) {
      final Command interrupter = entry.getKey();
      final Command interrupted = entry.getValue();

      interrupters[j] = getCommandName(interrupter) + " interrupted " + getCommandName(interrupted);
      j++;
    }

    Logger.recordOutput("CommandScheduler/Running/errors", interrupters);
  }

  public static void logRequiredSubsystems() {
    Logger.recordOutput("CommandScheduler/Subsystems/.type", "Alerts");

    final String[] subsystems = new String[requiredSubsystems.size()];
    {
      int i = 0;
      for (final Map.Entry<Subsystem, Command> entry : requiredSubsystems.entrySet()) {
        final Subsystem required = entry.getKey();
        final Command command = entry.getValue();

        subsystems[i] = required.getName() + " (" + command.getName() + ")";
        i++;
      }
    }
    Logger.recordOutput("CommandScheduler/Subsystems/infos", subsystems);
  }
}
