# 2026 Rebuilt

The code for Team 167 Children of the Corn's 2026 Rebuilt robot.

- Used libraries:
  - WPILib
    - [Docs](https://docs.wpilib.org/en/stable/index.html)
    - [Download](https://github.com/wpilibsuite/allwpilib/releases)
  - AdvantageKit
    - [Docs](https://docs.advantagekit.org/)
  - Phoenix 6
    - [Docs](https://v6.docs.ctr-electronics.com/en/latest/)

## Useful Gradle commands:

```sh
# Compile the code
# Equivalent to `WPILib: Build Robot Code` in VS Code command palette
$ ./gradlew build

# Run simulation
# Equivalent to `WPILib: Simulate Robot Code` in VS Code command palette
$ ./gradlew simulateJava

# Deploy code to robot
# Equivalent to `WPILib: Deploy Robot Code` in VS Code command palette
$ ./gradlew deploy

# Run replay - have log open in AdvantageScope
$ ./gradlew simulateJava -P replay

# Apply autoformatter (will also check syntax)
$ ./gradlew spotlessApply
```
