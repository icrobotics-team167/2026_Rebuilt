package frc.cotc;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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
    
    //Nonclimb Autos
    //Depot
    public AutoRoutine centerDepot() {
        AutoRoutine routine = autoFactory.newRoutine("CenterDepot");
        AutoTrajectory centerDepot = routine.trajectory("CenterDepot");

        routine.active().onTrue(Commands.sequence(centerDepot.resetOdometry(),centerDepot.cmd()));

        return routine;
    }

    public AutoRoutine leftDepot() {
        AutoRoutine routine = autoFactory.newRoutine("LeftOutpost");
        AutoTrajectory leftDepot = routine.trajectory("LeftOutpost");

        routine.active().onTrue(Commands.sequence(leftDepot.resetOdometry(),leftDepot.cmd()));

        return routine;
    }

    public AutoRoutine rightDepot() {
        AutoRoutine routine = autoFactory.newRoutine("RightOutpost");
        AutoTrajectory rightDepot = routine.trajectory("RightOutpost");

        routine.active().onTrue(Commands.sequence(rightDepot.resetOdometry(),rightDepot.cmd()));

        return routine;
    }

    //DepotOutpost
    public AutoRoutine centerDepotOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("CenterDepotOutpost");
        AutoTrajectory centerDepotOutpost = routine.trajectory("CenterDepotOutpost");

        routine.active().onTrue(Commands.sequence(centerDepotOutpost.resetOdometry(),centerDepotOutpost.cmd()));
        
        return routine;
    }

    public AutoRoutine leftDepotOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("LeftDepotOutpost");
        AutoTrajectory leftDepotOutpost = routine.trajectory("LeftDepotOutpost");

        routine.active().onTrue(Commands.sequence(leftDepotOutpost.resetOdometry(),leftDepotOutpost.cmd()));
        
        return routine;
    }

    public AutoRoutine rightDepotOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("RightDepotOutpost");
        AutoTrajectory rightDepotOutpost = routine.trajectory("RightDepotOutpost");

        routine.active().onTrue(Commands.sequence(rightDepotOutpost.resetOdometry(),rightDepotOutpost.cmd()));
        
        return routine;
    }

    //Fuel
    public AutoRoutine centerFuel() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuel");
        AutoTrajectory centerFuel = routine.trajectory("CenterFuel");
        
        routine.active().onTrue(Commands.sequence(centerFuel.resetOdometry(),centerFuel.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuel() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuel");
        AutoTrajectory leftFuel = routine.trajectory("LeftFuel");
        
        routine.active().onTrue(Commands.sequence(leftFuel.resetOdometry(),leftFuel.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuel() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuel");
        AutoTrajectory rightFuel = routine.trajectory("RightFuel");
        
        routine.active().onTrue(Commands.sequence(rightFuel.resetOdometry(),rightFuel.cmd()));
        
        return routine;
    }

    //FuelDepot
    public AutoRoutine centerFuelDepot() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelDepot");
        AutoTrajectory centerFuelDepot = routine.trajectory("CenterFuelDepot");
        
        routine.active().onTrue(Commands.sequence(centerFuelDepot.resetOdometry(),centerFuelDepot.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuelDepot() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuelDepot");
        AutoTrajectory leftFuelDepot = routine.trajectory("LeftFuelDepot");
        
        routine.active().onTrue(Commands.sequence(leftFuelDepot.resetOdometry(),leftFuelDepot.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuelDepot() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuelDepot");
        AutoTrajectory rightFuelDepot = routine.trajectory("RightFuelDepot");
        
        routine.active().onTrue(Commands.sequence(rightFuelDepot.resetOdometry(),rightFuelDepot.cmd()));
        
        return routine;
    }

    //FuelDepotOutpost
    public AutoRoutine centerFuelDepotOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelDepotOutpost");
        AutoTrajectory centerFuelDepotOutpost = routine.trajectory("CenterFuelDepotOutpost");
        
        routine.active().onTrue(Commands.sequence(centerFuelDepotOutpost.resetOdometry(),centerFuelDepotOutpost.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuelDepotOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuelDepotOutpost");
        AutoTrajectory leftFuelDepotOutpost = routine.trajectory("LeftFuelDepotOutpost");
        
        routine.active().onTrue(Commands.sequence(leftFuelDepotOutpost.resetOdometry(),leftFuelDepotOutpost.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuelDepotOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuelDepotOutpost");
        AutoTrajectory rightFuelDepotOutpost = routine.trajectory("RightFuelDepotOutpost");
        
        routine.active().onTrue(Commands.sequence(rightFuelDepotOutpost.resetOdometry(),rightFuelDepotOutpost.cmd()));
        
        return routine;
    }

    //FuelOutpost
    public AutoRoutine centerFuelOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelOutpost");
        AutoTrajectory centerFuelOutpost = routine.trajectory("CenterFuelOutpost");
        
        routine.active().onTrue(Commands.sequence(centerFuelOutpost.resetOdometry(),centerFuelOutpost.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuelOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuelOutpost");
        AutoTrajectory leftFuelOutpost = routine.trajectory("LeftFuelOutpost");
        
        routine.active().onTrue(Commands.sequence(leftFuelOutpost.resetOdometry(),leftFuelOutpost.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuelOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuelOutpost");
        AutoTrajectory rightFuelOutpost = routine.trajectory("RightFuelOutpost");
        
        routine.active().onTrue(Commands.sequence(rightFuelOutpost.resetOdometry(),rightFuelOutpost.cmd()));
        
        return routine;
    }

    //FuelOutpostDepot
    public AutoRoutine centerFuelOutpostDepot() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelOutpostDepot");
        AutoTrajectory centerFuelOutpostDepot = routine.trajectory("CenterFuelOutpostDepot");
        
        routine.active().onTrue(Commands.sequence(centerFuelOutpostDepot.resetOdometry(),centerFuelOutpostDepot.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuelOutpostDepot() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuelOutpostDepot");
        AutoTrajectory leftFuelOutpostDepot = routine.trajectory("LeftFuelOutpostDepot");
        
        routine.active().onTrue(Commands.sequence(leftFuelOutpostDepot.resetOdometry(),leftFuelOutpostDepot.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuelOutpostDepot() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuelOutpostDepot");
        AutoTrajectory rightFuelOutpostDepot = routine.trajectory("RightFuelOutpostDepot");
        
        routine.active().onTrue(Commands.sequence(rightFuelOutpostDepot.resetOdometry(),rightFuelOutpostDepot.cmd()));
        
        return routine;
    }
    
    //Outpost
    public AutoRoutine centerOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("CenterOutpost");
        AutoTrajectory centerOutpost = routine.trajectory("CenterOutpost");
        
        routine.active().onTrue(Commands.sequence(centerOutpost.resetOdometry(),centerOutpost.cmd()));
        
        return routine;
    }

    public AutoRoutine leftOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("LeftOutpost");
        AutoTrajectory leftOutpost = routine.trajectory("LeftOutpost");
        
        routine.active().onTrue(Commands.sequence(leftOutpost.resetOdometry(),leftOutpost.cmd()));
        
        return routine;
    }

    public AutoRoutine rightOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("RightOutpost");
        AutoTrajectory rightOutpost = routine.trajectory("RightOutpost");
        
        routine.active().onTrue(Commands.sequence(rightOutpost.resetOdometry(),rightOutpost.cmd()));
        
        return routine;
    }

    //OutpostDepot
    public AutoRoutine centerOutpostDepot() {
        AutoRoutine routine = autoFactory.newRoutine("CenterOutpostDepot");
        AutoTrajectory centerOutpostDepot = routine.trajectory("CenterOutpostDepot");
        
        routine.active().onTrue(Commands.sequence(centerOutpostDepot.resetOdometry(),centerOutpostDepot.cmd()));
        
        return routine;
    }

    public AutoRoutine leftOutpostDepot() {
        AutoRoutine routine = autoFactory.newRoutine("LeftOutpostDepot");
        AutoTrajectory leftOutpostDepot = routine.trajectory("LeftOutpostDepot");
        
        routine.active().onTrue(Commands.sequence(leftOutpostDepot.resetOdometry(),leftOutpostDepot.cmd()));
        
        return routine;
    }

    public AutoRoutine rightOutpostDepot() {
        AutoRoutine routine = autoFactory.newRoutine("RightOutpostDepot");
        AutoTrajectory rightOutpostDepot = routine.trajectory("RightOutpostDepot");
        
        routine.active().onTrue(Commands.sequence(rightOutpostDepot.resetOdometry(),rightOutpostDepot.cmd()));
        
        return routine;
    }
    


    //Climb Autos
    //Climb Only
    public AutoRoutine centerClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterClimb");
        AutoTrajectory driveCenterClimb = routine.trajectory("CenterClimb");
        
        routine.active().onTrue(Commands.sequence(driveCenterClimb.resetOdometry(), driveCenterClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftClimb");
        AutoTrajectory driveLeftClimb = routine.trajectory("LeftClimb");
        
        routine.active().onTrue(Commands.sequence(driveLeftClimb.resetOdometry(), driveLeftClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightClimb");
        AutoTrajectory driveRightClimb = routine.trajectory("RightClimb");
        
        routine.active().onTrue(Commands.sequence(driveRightClimb.resetOdometry(), driveRightClimb.cmd()));
        
        return routine;
    }

    //Depot Climb
    public AutoRoutine centerDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterDepotClimb");
        AutoTrajectory driveCenterDepotClimb = routine.trajectory("CenterDepotClimb");
        
        routine.active().onTrue(Commands.sequence(driveCenterDepotClimb.resetOdometry(), driveCenterDepotClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftDepotClimb");
        AutoTrajectory driveLeftDepotClimb = routine.trajectory("LeftDepotClimb");
        
        routine.active().onTrue(Commands.sequence(driveLeftDepotClimb.resetOdometry(), driveLeftDepotClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightDepotClimb");
        AutoTrajectory driveRightDepotClimb = routine.trajectory("RightDepotClimb");
        
        routine.active().onTrue(Commands.sequence(driveRightDepotClimb.resetOdometry(), driveRightDepotClimb.cmd()));
        
        return routine;
    }
    
    //Depot Outpost Climb
    public AutoRoutine centerDepotOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterDepotOutpostClimb");
        AutoTrajectory driveCenterDepotOutpostClimb = routine.trajectory("CenterDepotOutpostClimb");
        
        routine.active().onTrue(Commands.sequence(driveCenterDepotOutpostClimb.resetOdometry(), driveCenterDepotOutpostClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftDepotOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftDepotOutpostClimb");
        AutoTrajectory driveLeftDepotOutpostClimb = routine.trajectory("LeftDepotOutpostClimb");
        
        routine.active().onTrue(Commands.sequence(driveLeftDepotOutpostClimb.resetOdometry(), driveLeftDepotOutpostClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightDepotOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightDepotOutpostClimb");
        AutoTrajectory driveRightDepotOutpostClimb = routine.trajectory("RightDepotOutpostClimb");
        
        routine.active().onTrue(Commands.sequence(driveRightDepotOutpostClimb.resetOdometry(), driveRightDepotOutpostClimb.cmd()));
        
        return routine;
    }

    //Fuel Climb
    public AutoRoutine centerFuelClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelClimb");
        AutoTrajectory driveCenterFuelClimb = routine.trajectory("CenterFuelClimb");
        
        routine.active().onTrue(Commands.sequence(driveCenterFuelClimb.resetOdometry(), driveCenterFuelClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuelClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuelClimb");
        AutoTrajectory driveLeftFuelClimb = routine.trajectory("LeftFuelClimb");

        routine.active().onTrue(Commands.sequence(driveLeftFuelClimb.resetOdometry(), driveLeftFuelClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuelClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuelClimb");
        AutoTrajectory driveRightFuelClimb = routine.trajectory("RightFuelClimb");

        routine.active().onTrue(Commands.sequence(driveRightFuelClimb.resetOdometry(), driveRightFuelClimb.cmd()));

        return routine;
    }

    //Fuel Depot Climb
    public AutoRoutine centerFuelDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelDepotClimb");
        AutoTrajectory driveCenterFuelDepotClimb = routine.trajectory("CenterFuelDepotClimb");

        routine.active().onTrue(Commands.sequence(driveCenterFuelDepotClimb.resetOdometry(), driveCenterFuelDepotClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuelDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuelDepotClimb");
        AutoTrajectory driveLeftFuelDepotClimb = routine.trajectory("LeftFuelDepotClimb");

        routine.active().onTrue(Commands.sequence(driveLeftFuelDepotClimb.resetOdometry(), driveLeftFuelDepotClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuelDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuelDepotClimb");
        AutoTrajectory driveRightFuelDepotClimb = routine.trajectory("RightFuelDepotClimb");

        routine.active().onTrue(Commands.sequence(driveRightFuelDepotClimb.resetOdometry(), driveRightFuelDepotClimb.cmd()));
        
        return routine;
    }

    //Fuel Depot Outpost Climb
    public AutoRoutine centerFuelDepotOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelDepotOutpostClimb");
        AutoTrajectory driveCenterFuelDepotOutpostClimb = routine.trajectory("CenterFuelDepotOutpostClimb");

        routine.active().onTrue(Commands.sequence(driveCenterFuelDepotOutpostClimb.resetOdometry(), driveCenterFuelDepotOutpostClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuelDepotOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuelDepotOutpostClimb");
        AutoTrajectory driveLeftFuelDepotOutpostClimb = routine.trajectory("LeftFuelDepotOutpostClimb");

        routine.active().onTrue(Commands.sequence(driveLeftFuelDepotOutpostClimb.resetOdometry(), driveLeftFuelDepotOutpostClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuelDepotOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuelDepotOutpostClimb");
        AutoTrajectory driveRightFuelDepotOutpostClimb = routine.trajectory("RightFuelDepotOutpostClimb");

        routine.active().onTrue(Commands.sequence(driveRightFuelDepotOutpostClimb.resetOdometry(), driveRightFuelDepotOutpostClimb.cmd()));
        
        return routine;
    }

    //Fuel Outpost Climb
    public AutoRoutine centerFuelOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelOutpostClimb");
        AutoTrajectory driveCenterFuelOutpostClimb = routine.trajectory("CenterFuelOutpostClimb");

        routine.active().onTrue(Commands.sequence(driveCenterFuelOutpostClimb.resetOdometry(), driveCenterFuelOutpostClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuelOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuelOutpostClimb");
        AutoTrajectory driveLeftFuelOutpostClimb = routine.trajectory("LeftFuelOutpostClimb");

        routine.active().onTrue(Commands.sequence(driveLeftFuelOutpostClimb.resetOdometry(), driveLeftFuelOutpostClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuelOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuelOutpostClimb");
        AutoTrajectory driveRightFuelOutpostClimb = routine.trajectory("RightFuelOutpostClimb");

        routine.active().onTrue(Commands.sequence(driveRightFuelOutpostClimb.resetOdometry(), driveRightFuelOutpostClimb.cmd()));
        
        return routine;
    }

    //Fuel Outpost Depot Climb
    public AutoRoutine centerFuelOutpostDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelOutpostDepotClimb");
        AutoTrajectory driveCenterFuelOutpostDepotClimb = routine.trajectory("CenterFuelOutpostDepotClimb");

        routine.active().onTrue(Commands.sequence(driveCenterFuelOutpostDepotClimb.resetOdometry(), driveCenterFuelOutpostDepotClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftFuelOutpostDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftFuelOutpostDepotClimb");
        AutoTrajectory driveLeftFuelOutpostDepotClimb = routine.trajectory("LeftFuelOutpostDepotClimb");

        routine.active().onTrue(Commands.sequence(driveLeftFuelOutpostDepotClimb.resetOdometry(), driveLeftFuelOutpostDepotClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightFuelOutpostDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightFuelOutpostDepotClimb");
        AutoTrajectory driveRightFuelOutpostDepotClimb = routine.trajectory("RightFuelOutpostDepotClimb");

        routine.active().onTrue(Commands.sequence(driveRightFuelOutpostDepotClimb.resetOdometry(), driveRightFuelOutpostDepotClimb.cmd()));
        
        return routine;
    }

    //Outpost Climb
    public AutoRoutine centerOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterOutpostClimb");
        AutoTrajectory driveCenterOutpostClimb = routine.trajectory("CenterOutpostClimb");
        
        routine.active().onTrue(Commands.sequence(driveCenterOutpostClimb.resetOdometry(), driveCenterOutpostClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftOutpostClimb");
        AutoTrajectory driveLeftOutpostClimb = routine.trajectory("LeftOutpostClimb");
        
        routine.active().onTrue(Commands.sequence(driveLeftOutpostClimb.resetOdometry(), driveLeftOutpostClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightOutpostClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightOutpostClimb");
        AutoTrajectory driveRightOutpostClimb = routine.trajectory("RightOutpostClimb");
        
        routine.active().onTrue(Commands.sequence(driveRightOutpostClimb.resetOdometry(), driveRightOutpostClimb.cmd()));
        
        return routine;
    }

    //Outpost Depot Climb
    public AutoRoutine centerOutpostDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("CenterOutpostDepotClimb");
        AutoTrajectory driveCenterOutpostDepotClimb = routine.trajectory("CenterOutpostDepotClimb");
        
        routine.active().onTrue(Commands.sequence(driveCenterOutpostDepotClimb.resetOdometry(), driveCenterOutpostDepotClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine leftOutpostDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("LeftOutpostDepotClimb");
        AutoTrajectory driveLeftOutpostDepotClimb = routine.trajectory("LeftOutpostDepotClimb");
        
        routine.active().onTrue(Commands.sequence(driveLeftOutpostDepotClimb.resetOdometry(), driveLeftOutpostDepotClimb.cmd()));
        
        return routine;
    }

    public AutoRoutine rightOutpostDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("RightOutpostDepotClimb");
        AutoTrajectory driveRightOutpostDepotClimb = routine.trajectory("RightOutpostDepotClimb");
        
        routine.active().onTrue(Commands.sequence(driveRightOutpostDepotClimb.resetOdometry(), driveRightOutpostDepotClimb.cmd()));
        
        return routine;
    }
}
