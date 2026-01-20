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
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 * Implementation that extends the real implementation to add MapleSim-based drivetrain simulation.
 */
public class SwerveIOSim extends SwerveIOReal {
  private final SwerveDriveSimulation simulation;
  private final Pigeon2SimState pigeonSim;

  @SuppressWarnings("FieldCanBeLocal")
  private final double SIM_FREQUENCY_HZ = 250.0;

  private final Pose2d INIT_POSE = new Pose2d(7, 2, Rotation2d.fromDegrees(120));

  @SuppressWarnings("unchecked")
  public SwerveIOSim() {
    // Tuner constants need to be adjusted for simulation to avoid some MapleSim bugs
    super(
        adjustConstants(TunerConstants.FrontLeft),
        adjustConstants(TunerConstants.FrontRight),
        adjustConstants(TunerConstants.BackLeft),
        adjustConstants(TunerConstants.BackRight));

    // Initialize drivetrain simulation
    simulation =
        new SwerveDriveSimulation(
            new DriveTrainSimulationConfig(
                Pounds.of(140),
                //for bumper thickness of 2.5in
                Inches.of(27), // Bumper width
                Inches.of(37), // Bumper length
                // Track width
                getModuleLocations()[0].getMeasureY().minus(getModuleLocations()[1].getMeasureY()),
                // Track length
                getModuleLocations()[0].getMeasureX().minus(getModuleLocations()[2].getMeasureX()),
                COTS.ofPigeon2(),
                // Module specs
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
            INIT_POSE);
    // Set up motor controller sims for each module
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
              encoderSimState.setRawPosition(mechanismAngle);
              encoderSimState.setVelocity(mechanismVelocity);
              encoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
              motorSimState.setRawRotorPosition(encoderAngle);
              motorSimState.setRotorVelocity(encoderVelocity);
              motorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

              return motorSimState.getMotorVoltageMeasure();
            }
          });
    }
    pigeonSim = getPigeon2().getSimState();
    // The default friction value for MapleSim is unrealistically high.
    // This needs tuning to the real robot.
    simulation.setLinearDamping(0.1);
    simulation.setAngularDamping(0.1);

    // Run the simulation at a higher frequency than the default
    SimulatedArena.overrideSimulationTimings(Seconds.of(1.0 / SIM_FREQUENCY_HZ), 1);
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    SimulatedArena.getInstance().addDriveTrainSimulation(simulation);
    var simThread = new Notifier(this::update);
    simThread.startPeriodic(1.0 / SIM_FREQUENCY_HZ);

    Robot.groundTruthPoseSupplier = simulation::getSimulatedDriveTrainPose;
  }

  private void update() {
    // Update the simulation one timestep forwards
    SimulatedArena.getInstance().simulationPeriodic();
    // Update the gyro sim
    pigeonSim.setRawYaw(
        simulation
            .getSimulatedDriveTrainPose()
            .getRotation()
            .minus(INIT_POSE.getRotation())
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
        // MapleSim doesn't simulate coupling ratio
        .withCouplingGearRatio(0)
        // Adjust steer motor PID gains for simulation
        .withSteerMotorGains(moduleConstants.SteerMotorGains.withKP(70).withKD(.5));
  }
}
