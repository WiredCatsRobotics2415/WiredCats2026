package frc.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Represents a subsystem responsible for controlling the drive system of a robot. This includes methods for controlling
 * the movement, managing swerve drive modules, tracking the robot's position and orientation, and other related tasks.
 */
public interface SwerveDrive extends Subsystem {
    double kMaxAngularSpeed = 0;
    double kMaxSpeed = 0;

    /** Called every 20ms by the scheduler */
    void periodic();

    /** Command-friendly drive method (expects m/s and rad/s). */
    void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative);

    void stop();

    Pose2d getPose();

    void resetPose(Pose2d pose);
}