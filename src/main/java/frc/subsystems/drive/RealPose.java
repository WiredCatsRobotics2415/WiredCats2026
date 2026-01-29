package frc.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.subsystems.vision.Vision;
import frc.utils.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

/**
 * Handles vision-odometry fusion for real robot drivetrain (Phoenix 6 Swerve).
 * Works with CommandSwerveDrivetrain which uses Phoenix 6 SwerveDrivetrain.
 */
public class RealPose {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    // Gyro rate threshold for rejecting vision updates (degrees per second)
    private static final double MAX_GYRO_RATE_DEG_PER_SEC = 360.0;

    // Vision standard deviations based on tag count and distance
    private static final double SINGLE_TAG_XY_STD_DEV = 0.9;
    private static final double MULTI_TAG_XY_STD_DEV = 0.5;
    private static final double ROTATION_STD_DEV = 9999999; // Trust gyro over vision

    public RealPose(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
    }

    /**
     * Updates odometry with vision measurements. Should be called periodically.
     */
    public void updateOdometryAndVision() {
        // 1. Get current drivetrain state from Phoenix 6
        var drivetrainState = drivetrain.getState();
        Pose2d currentPose = drivetrainState.Pose;

        // Get gyro rate from Pigeon2
        double gyroRate = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        gyroRate = Math.toDegrees(gyroRate); // Convert from rad/s to deg/s

        // 2. Send robot orientation to vision subsystem (required for MegaTag2)
        vision.sendOrientation(
            currentPose.getRotation().getDegrees(),
            gyroRate
        );

        // 3. Get vision estimates from all cameras
        PoseEstimate[] visionEstimates = vision.getPoseEstimates();

        // 4. Check if robot is spinning too fast (reject vision if unstable)
        boolean rejectUpdate = Math.abs(gyroRate) > MAX_GYRO_RATE_DEG_PER_SEC;

        if (rejectUpdate) {
            Logger.recordOutput("Drive/VisionRejected", true);
            return;
        }

        Logger.recordOutput("Drive/VisionRejected", false);

        // 5. Fuse vision measurements into Phoenix 6's pose estimator
        for (int i = 0; i < visionEstimates.length; i++) {
            PoseEstimate estimate = visionEstimates[i];

            // Only add measurement if we detected at least one AprilTag
            if (estimate.tagCount > 0) {
                // Calculate standard deviations based on tag count and distance
                double xyStdDev = calculateXYStdDev(estimate);
                double rotStdDev = ROTATION_STD_DEV;

                // Add the vision measurement to Phoenix 6's pose estimator
                drivetrain.addVisionMeasurement(
                    estimate.pose,
                    estimate.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
                );

                // Log the vision measurement
                Logger.recordOutput("Drive/VisionPose" + i, estimate.pose);
                Logger.recordOutput("Drive/VisionTagCount" + i, estimate.tagCount);
                Logger.recordOutput("Drive/VisionXYStdDev" + i, xyStdDev);

                if (estimate.avgTagDist > 0) {
                    Logger.recordOutput("Drive/VisionAvgTagDist" + i, estimate.avgTagDist);
                }
            }
        }

        // Log current pose
        Logger.recordOutput("Drive/EstimatedPose", currentPose);
        Logger.recordOutput("Drive/GyroRate", gyroRate);
    }

    /**
     * Calculates XY standard deviation based on tag count and average distance.
     * More tags and closer distance = lower standard deviation (more trust).
     */
    private double calculateXYStdDev(PoseEstimate estimate) {
        double baseStdDev;

        // Base standard deviation on tag count
        if (estimate.tagCount >= 2) {
            baseStdDev = MULTI_TAG_XY_STD_DEV;
        } else {
            baseStdDev = SINGLE_TAG_XY_STD_DEV;
        }

        // Increase standard deviation with distance (if available)
        if (estimate.avgTagDist > 0) {
            // For every meter of distance, increase std dev by 0.1m
            // This makes us trust close measurements more than far ones
            double distanceFactor = 1.0 + (estimate.avgTagDist * 0.1);
            baseStdDev *= distanceFactor;
        }

        return baseStdDev;
    }
}
