package frc.cotc.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class HoodIOPhoenix implements HoodIO {

    private final TalonFX motor;

    private final BaseStatusSignal posSignal, velSignal, statorSignal, supplySignal;

    public HoodIOPhoenix() {
        motor = new TalonFX(0); // TODO: Get the actual ID
        var encoder = new CANcoder(0); // TODO: Get the actual ID

        posSignal = encoder.getAbsolutePosition();
        velSignal = motor.getVelocity();
        statorSignal = motor.getStatorCurrent();
        supplySignal = motor.getSupplyCurrent();
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
