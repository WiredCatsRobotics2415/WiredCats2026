package frc.robot.subsystems.drive;

/**
 * Represents a subsystem responsible for controlling the drive system of a robot. This includes methods for controlling
 * the movement, managing swerve drive modules, tracking the robot's position and orientation, and other related tasks.
 */
public interface SwerveDrive extends Subsystem {
    void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop);

    void setModuleStates(SwerveModuleState[] desiredStates);

    ChassisSpeeds getMeasuredSpeeds();

    Rotation2d getGyroYaw();

    Pose2d getPose();

    void setPose(Pose2d pose);

    default Rotation2d getHeading() {
        return getPose().getRotation();
    }

    default void setHeading(Rotation2d heading) {
        setPose(new Pose2d(getPose().getTranslation(), heading));
    }

    default void zeroHeading() {
        setHeading(new Rotation2d());
    }

    void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);
    void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
}