package frc.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.subsystems.vision.Vision;
import frc.utils.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

/**
 * Handles vision-odometry fusion for simulated drivetrain (MapleSimSwerveDrivetrain).
 * Works with CommandSwerveDrivetrain which uses Phoenix 6 SwerveDrivetrain with MapleSimSwerveDrivetrain.
 */
public class SimulatePose {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    public SimulatePose(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
    }

    /**
     * Updates odometry with vision measurements. Should be called periodically.
     */
    public void updateOdometryAndVision() {
        // 1. Get current pose from Phoenix 6 drivetrain (already fused with MapleSimSwerveDrivetrain)
        Pose2d currentPose = drivetrain.getState().Pose;

        // 2. Send robot orientation to vision subsystem (required for MegaTag2)
        // In simulation, gyro rate is always 0 (stable)
        vision.sendOrientation(
            currentPose.getRotation().getDegrees(),
            0.0  // No gyro rate in sim (always stable)
        );

        // 3. Get vision estimates from all cameras
        PoseEstimate[] visionEstimates = vision.getPoseEstimates();

        // 4. Sim is always stable (no spinning), so we never reject updates
        boolean rejectUpdate = false;

        // 5. Fuse vision measurements into Phoenix 6 pose estimator
        if (!rejectUpdate) {
            for (int i = 0; i < visionEstimates.length; i++) {
                PoseEstimate estimate = visionEstimates[i];

                // Only add measurement if we detected at least one AprilTag
                if (estimate.tagCount > 0) {
                    // Set standard deviations based on distance and tag count
                    // More tags = more trust, farther distance = less trust
                    double xyStdDev = 0.5; // Base standard deviation in meters
                    double rotStdDev = 9999999; // Effectively ignore rotation from vision

                    // Adjust trust based on number of tags
                    if (estimate.tagCount >= 2) {
                        xyStdDev = 0.3; // More trust with multiple tags
                    }

                    // Add the vision measurement to Phoenix 6's pose estimator
                    drivetrain.addVisionMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds,
                        VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
                    );

                    // Log the vision measurement
                    Logger.recordOutput("Drive/VisionPose" + i, estimate.pose);
                    Logger.recordOutput("Drive/VisionTagCount" + i, estimate.tagCount);
                }
            }
        }

        // Log current pose
        Logger.recordOutput("Drive/EstimatedPose", drivetrain.getState().Pose);
    }
}
