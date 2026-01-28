package frc.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Represents a subsystem responsible for controlling the drive system of a robot. This includes methods for controlling
 * the movement, managing swerve drive modules, tracking the robot's position and orientation, and other related tasks.
 */
public interface SwerveDrive extends Subsystem {
    double kMaxAngularSpeed = 3;
    double kMaxSpeed = 3;

    /** Called every 20ms by the scheduler */
    void periodic();

    /** Command-friendly drive method (expects m/s for x and y speeds and rad/s). */
    void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative);

    void stop();

    Pose2d getPose();

    void resetPose(Pose2d pose);

    /**
     * Returns the ground truth pose for simulation purposes.
     * For real robot implementations, this returns the same as getPose() since ground truth is not available.
     * For simulation implementations, this should return the actual simulated pose without any estimation.
     * @return The ground truth pose (simulation) or fused pose (real robot)
     */
    default Pose2d getGroundTruthPose() {
        return getPose();
    }
}