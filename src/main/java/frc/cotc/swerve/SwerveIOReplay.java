package frc.cotc.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;

public class SwerveIOReplay  extends TunerConstants.TunerSwerveDrivetrain implements SwerveIO {
  private final SwerveDrivePoseEstimator poseEstimator;
  private boolean poseEstInit = false;

  public SwerveIOReplay(
    SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);

    poseEstimator =
      new SwerveDrivePoseEstimator(
        getKinematics(),
        Rotation2d.kZero,
        new SwerveModulePosition[modules.length],
        Pose2d.kZero);
  }

  @Override
  public void updateOdometry(SwerveIOInputs inputs) {
    if (!poseEstInit) {
      poseEstimator.resetPosition(
        inputs.rawHeadingQueue[0], inputs.modulePositionsQueue[0], inputs.poseQueue[0]);
      poseEstInit = true;
    }

    for (int i = 0; i < inputs.timestampQueue.length; ++i) {
      // Apply update
      poseEstimator.updateWithTime(
        inputs.timestampQueue[i], inputs.rawHeadingQueue[i], inputs.modulePositionsQueue[i]);
    }
  }

  @Override
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void tareEverything() {
    resetPose(Pose2d.kZero);
  }

  @Override
  public void seedFieldCentric() {
    poseEstimator.resetRotation(getOperatorForwardDirection());
  }

  @Override
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  @Override
  public void resetTranslation(Translation2d translation) {
    poseEstimator.resetTranslation(translation);
  }

  @Override
  public void resetRotation(Rotation2d rotation) {
    poseEstimator.resetRotation(rotation);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(
      visionRobotPoseMeters, timestampSeconds);
  }

  @Override
  public void addVisionMeasurement(
    Pose2d visionRobotPoseMeters,
    double timestampSeconds,
    Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
      visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @Override
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }

  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return poseEstimator.sampleAt(timestampSeconds);
  }
}
