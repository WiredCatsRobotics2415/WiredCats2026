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
import frc.subsystems.vision.Vision;
import frc.utils.LimelightHelpers.PoseEstimate;

public class SwerveDriveReal implements SwerveDrive {
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
  private final Vision m_vision;

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

  public SwerveDriveReal(Vision vision) {
    this.m_vision = vision;
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
    // 1. Update odometry with wheel encoders + gyro
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });

    // 2. Send robot orientation to vision subsystem (required for MegaTag2)
    m_vision.sendOrientation(getPose().getRotation().getDegrees(), m_gyro.getRate());

    // 3. Get pose estimates from all 3 Limelights
    PoseEstimate[] visionEstimates = m_vision.getPoseEstimates();

    // 4. Check if robot is spinning too fast (reject vision if unstable)
    boolean rejectUpdate = Math.abs(m_gyro.getRate()) > 360; // deg/sec

    // 5. Fuse vision measurements into pose estimator
    if (!rejectUpdate) {
      for (int i = 0; i < visionEstimates.length; i++) {
        PoseEstimate estimate = visionEstimates[i];

        // Only add measurement if we detected at least one AprilTag
        if (estimate.tagCount > 0) {
          // Set standard deviations (trust vision for position, ignore rotation)
          // X/Y: 0.7m uncertainty, Rotation: effectively infinite (trust gyro instead)
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));

          // Add the vision measurement with its timestamp
          m_poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
        }
      }
    }
  }
}
