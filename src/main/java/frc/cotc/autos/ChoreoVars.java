// spotless:off
package frc.cotc.autos;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final LinearVelocity BumpCrossSpeed = Units.MetersPerSecond.of(2);
    public static final Distance BumpExitX = Units.Meters.of(5.6);
    public static final LinearVelocity DepotIntakeSpeed = Units.MetersPerSecond.of(1);
    public static final double DepotLeadOutShoot = 1;
    public static final Distance FieldWidth = Units.Meters.of(8.069);
    public static final LinearVelocity IntakeSpeed = Units.MetersPerSecond.of(1.5);
    public static final LinearVelocity WallSlamSpeed = Units.MetersPerSecond.of(1);

    public static final class Poses {
        public static final Pose2d CenterLeadOut = new Pose2d(3.0704, 4.0345, Rotation2d.fromRadians(0));
        public static final Pose2d CenterStartPose = new Pose2d(3.68, 4.0345, Rotation2d.fromRadians(0));
        public static final Pose2d Depot = new Pose2d(0.25, 5.95, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d DepotLeadIn = new Pose2d(1.5, 5.95, Rotation2d.fromRadians(0));
        public static final Pose2d DepotLeadOut = new Pose2d(1.2, 5.95, Rotation2d.fromRadians(0));
        public static final Pose2d LeftBumpCrossPose = new Pose2d(5.6, 5.569, Rotation2d.fromRadians(0));
        public static final Pose2d LeftBumpFarEnter = new Pose2d(8.5, 6.069, Rotation2d.fromRadians(-1.7453293));
        public static final Pose2d LeftBumpMidEnter = new Pose2d(7.7, 5.569, Rotation2d.fromRadians(-1.7453293));
        public static final Pose2d LeftBumpMidLeadIn = new Pose2d(6.75, 6.019, Rotation2d.fromRadians(0));
        public static final Pose2d LeftBumpStartPose = new Pose2d(3.68, 5.569, Rotation2d.fromRadians(0));
        public static final Pose2d LeftCloseEnter = new Pose2d(6, 6.969, Rotation2d.fromRadians(-1.7453293));
        public static final Pose2d LeftFarCenterExit = new Pose2d(8.5, 4.769, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d LeftFarEnter = new Pose2d(8.5, 6.969, Rotation2d.fromRadians(-1.7453293));
        public static final Pose2d LeftMidCenterExit = new Pose2d(7.7, 4.769, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d LeftMidEnter = new Pose2d(7.7, 6.969, Rotation2d.fromRadians(-1.7453293));
        public static final Pose2d LeftShoot = new Pose2d(2.527, 5.121, Rotation2d.fromRadians(-2.1167353));
        public static final Pose2d LeftTrenchOut = new Pose2d(5.47, 7.569, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d LeftTrenchStartPose = new Pose2d(4.47, 7.719, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d Outpost = new Pose2d(0.56, 0.68, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d OutpostLeadIn = new Pose2d(1, 0.68, Rotation2d.fromRadians(0));
        public static final Pose2d RightBumpCrossPose = new Pose2d(5.6, 2.5, Rotation2d.fromRadians(0));
        public static final Pose2d RightBumpFarEnter = new Pose2d(8.5, 2, Rotation2d.fromRadians(1.7453293));
        public static final Pose2d RightBumpMidEnter = new Pose2d(7.7, 2.5, Rotation2d.fromRadians(1.7453293));
        public static final Pose2d RightBumpMidLeadIn = new Pose2d(6.75, 2.05, Rotation2d.fromRadians(0));
        public static final Pose2d RightBumpStartPose = new Pose2d(3.68, 2.5, Rotation2d.fromRadians(0));
        public static final Pose2d RightCloseEnter = new Pose2d(6, 1.1, Rotation2d.fromRadians(1.7453293));
        public static final Pose2d RightFarAcrossExit = new Pose2d(8.5, 5.5, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d RightFarCenterExit = new Pose2d(8.5, 3.3, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d RightFarEnter = new Pose2d(8.5, 1.1, Rotation2d.fromRadians(1.7453293));
        public static final Pose2d RightMidAcrossExit = new Pose2d(7.7, 5.5, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d RightMidCenterExit = new Pose2d(7.7, 3.3, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d RightMidEnter = new Pose2d(7.7, 1.1, Rotation2d.fromRadians(1.7453293));
        public static final Pose2d RightShoot = new Pose2d(2.424, 2.776, Rotation2d.fromRadians(-1.1126474));
        public static final Pose2d RightTrenchOut = new Pose2d(5.47, 0.5, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d RightTrenchStartPose = new Pose2d(4.47, 0.35, Rotation2d.fromRadians(1.5707963));
    }
}
// spotless:on
