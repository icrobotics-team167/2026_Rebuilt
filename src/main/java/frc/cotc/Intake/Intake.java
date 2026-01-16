package frc.cotc.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.cotc.Intake.IntakeIO.IntakeIOInputs;

public class Intake {
    private final IntakeIO io;
    private final IntakeIOInputs inputs;
    private final DigitalInput beamBreakSensor;
    private final int BEAM_BREAK_SENSOR_ID = 0;//Placeholder ID for beamBreakSensor

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public Intake() {
        beamBreakSensor = new DigitalInput(BEAM_BREAK_SENSOR_ID);
    }

    public Command intake() {
        return Commands.parallel(run(() -> intakeMotor1.setVoltage()), run(() -> intakeMotor2.setVoltage()));
    }
    
    public Command outtake() {
        return Commands.parallel(runReverse(() -> intakeMotor1.setVoltage()), runReverse(() -> intakeMotor2.setVoltage()));
    }

    public Command getIntakeCommand() {
        return run(io::run).finallyDo(io::stop);
    }

    public Command getIntakeOutCommand() {
        return run(io::runReverse).finallyDo(io::stop);
    }

    public boolean isRunning() {
        return inputs.isRunning;
    }

    public boolean hasGamePiece() {
        return !beamBreakSensor.get();
      }

    public Command outtakeUntilGamepiece() {
        return outtake().until(this::hasGamePiece).withName("Outtake Until Gamepiece");
    }
    

}
