package frc.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LimelightHelpers;

public class SwerveDrive extends SubsystemBase {
  public static final double kMaxSpeed = 3.0;
  public static final double kMaxAngularSpeed = Math.PI;

  private final Translation2d m_frontLeftLocation  = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation   = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation  = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft  = new SwerveModule(1, 2, 0, 1, 2, 3);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
  private final SwerveModule m_backLeft   = new SwerveModule(5, 6, 8, 9, 10, 11);
  private final SwerveModule m_backRight  = new SwerveModule(7, 8, 12, 13, 14, 15);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          Pose2d.kZero,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public SwerveDrive() {
    m_gyro.reset();
  }

  /** Called every 20ms by the scheduler */
  @Override
  public void periodic() {
    updateOdometryAndVision();
  }

  /** Command-friendly drive method (expects m/s and rad/s). */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // 0.02 is typical; better: use TimedRobot period, but this is fine to start.
    double periodSeconds = 0.02;

    var states =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getPose().getRotation())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public void stop() {
    drive(0, 0, 0, false);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

  private void updateOdometryAndVision() {
    // Odometry update
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });

    // --- Your Limelight fusion, moved here ---
    LimelightHelpers.SetRobotOrientation(
        "limelight", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    var ll1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    LimelightHelpers.SetRobotOrientation(
        "limelight-2", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    var ll2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-2");

    LimelightHelpers.SetRobotOrientation(
        "limelight-3", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    var ll3 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-3");

    boolean rejectUpdate = Math.abs(m_gyro.getRate()) > 360;

    if (!rejectUpdate) {
      if (ll1.tagCount > 0) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        m_poseEstimator.addVisionMeasurement(ll1.pose, ll1.timestampSeconds);
      }
      if (ll2.tagCount > 0) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        m_poseEstimator.addVisionMeasurement(ll2.pose, ll2.timestampSeconds);
      }
      if (ll3.tagCount > 0) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        m_poseEstimator.addVisionMeasurement(ll3.pose, ll3.timestampSeconds);
      }
    }
  }
}
