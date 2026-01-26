package frc.cotc;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.cotc.swerve.Swerve;

public class Autos {
    private final AutoFactory autoFactory;

    public Autos(Swerve swerve) {
        autoFactory = new AutoFactory(swerve::getPose, 
                                      swerve::resetPose, 
                                      swerve::followPath,
                                    true, 
                                    swerve);
    }

}
