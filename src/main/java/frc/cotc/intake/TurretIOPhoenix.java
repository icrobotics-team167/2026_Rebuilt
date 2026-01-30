package frc.cotc.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.cotc.shooter.TurretIO;

public class TurretIOPhoenix implements TurretIO {
    private final CANcoder encoder;
    private final TalonFX motor;
    private final int TURRET_ID = 0; // placeholder
    private final int ENCODER_ID = 1; // placeHolder
    private final int TURRET_DEFAULT_VOLTAGE = 12;
    private final BaseStatusSignal posSignal, velSignal, statorSignal, supplySignal;

    public TurretIOPhoenix() {
        encoder = new CANcoder(ENCODER_ID);
        motor = new TalonFX(TURRET_ID);
        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        motor.getConfigurator().apply(config);

        posSignal = encoder.getAbsolutePosition(false);
        
    }

    @Override
    public void run() {
        motor.setVoltage(TURRET_DEFAULT_VOLTAGE);
    }

    @Override
    public void updateInputs() {

    }

    @Override
    public void stop() {
        motor.setVoltage(0);
    }
}
