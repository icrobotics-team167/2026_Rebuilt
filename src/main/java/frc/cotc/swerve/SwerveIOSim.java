// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import frc.cotc.Robot;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class SwerveIOSim extends SwerveIOReal {
  private final SwerveDriveSimulation simulation;
  private final Pigeon2SimState pigeonSim;

  private final Notifier simThread = new Notifier(this::update);
  private final double simFrequencyHz = 250.0;

  private final Pose2d initPose = new Pose2d(7, 2, Rotation2d.fromDegrees(120));

  public SwerveIOSim() {
    super(
        adjustConstants(TunerConstants.FrontLeft),
        adjustConstants(TunerConstants.FrontRight),
        adjustConstants(TunerConstants.BackLeft),
        adjustConstants(TunerConstants.BackRight));

    simulation =
        new SwerveDriveSimulation(
            new DriveTrainSimulationConfig(
                Pounds.of(140),
                Inches.of(25),
                Inches.of(25),
                getModuleLocations()[0].getMeasureY().minus(getModuleLocations()[1].getMeasureY()),
                getModuleLocations()[0].getMeasureX().minus(getModuleLocations()[2].getMeasureX()),
                COTS.ofPigeon2(),
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60Foc(1),
                    DCMotor.getKrakenX60Foc(1),
                    TunerConstants.kDriveGearRatio,
                    TunerConstants.kSteerGearRatio,
                    TunerConstants.kDriveFrictionVoltage,
                    TunerConstants.kSteerFrictionVoltage,
                    TunerConstants.kWheelRadius,
                    TunerConstants.kSteerInertia,
                    1.5)),
            initPose);
    for (int i = 0; i < 4; i++) {
      int I = i;
      simulation.getModules()[i].useDriveMotorController(
          new SimulatedMotorController() {
            private final TalonFXSimState simState = getModule(I).getDriveMotor().getSimState();

            @Override
            public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
              simState.setRawRotorPosition(encoderAngle);
              simState.setRotorVelocity(encoderVelocity);
              simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

              return simState.getMotorVoltageMeasure();
            }
          });
      simulation.getModules()[i].useSteerMotorController(
          new SimulatedMotorController() {
            private final CANcoderSimState encoderSimState =
                getModule(I).getEncoder().getSimState();
            private final TalonFXSimState motorSimState =
                getModule(I).getSteerMotor().getSimState();

            @Override
            public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
              encoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
              encoderSimState.setRawPosition(mechanismAngle);
              encoderSimState.setVelocity(mechanismVelocity);
              motorSimState.setRawRotorPosition(encoderAngle);
              motorSimState.setRotorVelocity(encoderVelocity);
              motorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

              return motorSimState.getMotorVoltageMeasure();
            }
          });
    }
    pigeonSim = getPigeon2().getSimState();
    simulation.setLinearDamping(0.1);
    simulation.setAngularDamping(0.1);

    SimulatedArena.overrideSimulationTimings(Seconds.of(1.0 / simFrequencyHz), 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(simulation);
    var simThread = new Notifier(this::update);
    simThread.startPeriodic(1.0 / simFrequencyHz);

    Robot.groundTruthPoseSupplier = simulation::getSimulatedDriveTrainPose;
  }

  private void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    pigeonSim.setRawYaw(
        simulation
            .getSimulatedDriveTrainPose()
            .getRotation()
            .minus(initPose.getRotation())
            .getMeasure());
    pigeonSim.setAngularVelocityZ(
        RadiansPerSecond.of(
            simulation.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
  }

  private static SwerveModuleConstants<?, ?, ?> adjustConstants(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    // Apply simulation-specific adjustments to module constants
    return moduleConstants
        // Disable encoder offsets
        .withEncoderOffset(0)
        // Disable motor inversions for drive and steer motors
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        // Disable CANCoder inversion
        .withEncoderInverted(false)
        // Adjust steer motor PID gains for simulation
        .withSteerMotorGains(moduleConstants.SteerMotorGains.withKP(70).withKD(.5));
  }
}
