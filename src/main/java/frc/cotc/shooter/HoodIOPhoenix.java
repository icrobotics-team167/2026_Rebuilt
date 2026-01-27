package frc.cotc.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.cotc.swerve.TunerConstants;

public class HoodIOPhoenix implements HoodIO {

    private final TalonFX motor;

    private final int HOOD_MOTOR_ID = 0; // TODO: Get the actual ID
    private final int HOOD_ENCODER_ID = 0; // TODO: Get the actual ID

    private final BaseStatusSignal posSignal, velSignal, statorSignal, supplySignal;

    public HoodIOPhoenix() {
        motor = new TalonFX(HOOD_MOTOR_ID); 
        CANcoder encoder = new CANcoder(HOOD_ENCODER_ID); 



        var encoderConfig = new CANcoderConfiguration();
        encoder.getConfigurator().apply(encoderConfig);

        posSignal = encoder.getAbsolutePosition(false);
        velSignal = motor.getVelocity(false);
        statorSignal = motor.getStatorCurrent(false);
        supplySignal = motor.getSupplyCurrent(false);
    }

    @Override
    public void updateInputs(HoodIOInputs hoodIOInputs) {
        hoodIOInputs.thetaRad = Units.rotationsToRadians(posSignal.getValueAsDouble());
        hoodIOInputs.omegaRadPerSec = Units.rotationsToRadians(velSignal.getValueAsDouble());
        hoodIOInputs.motorStatorCurrentAmps = statorSignal.getValueAsDouble();
        hoodIOInputs.motorSupplyCurrentAmps = supplySignal.getValueAsDouble();
    }


    private final PositionVoltage positionControlSignal = new PositionVoltage(0);

    @Override
    public void runPitch(double thetaRad, double omegaRadPerSec) {
        motor.setControl(positionControlSignal
            .withPosition(Units.radiansToRotations(thetaRad))
            .withVelocity(Units.radiansToRotations(omegaRadPerSec))
        );
    }

}
