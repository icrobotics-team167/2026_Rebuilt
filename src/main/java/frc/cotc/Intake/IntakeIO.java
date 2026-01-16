package frc.cotc.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void run() {}

    public default void runReverse() {}

    public default void stop() {}

    @AutoLog
    public class IntakeIOInputs {
        public double appleidVolts1 = 0.0;
        public double statorCurrentAmps1 = 0.0;
        public double supplyCurrentAmps1 = 0.0;
        public double appleidVolts2 = 0.0;
        public double statorCurrentAmps2 = 0.0;
        public double supplyCurrentAmps2 = 0.0;
    }
}
