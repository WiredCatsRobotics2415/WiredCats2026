package frc.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.vision.Vision.TurretPipeline;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public Pose2d[] poseEstimates = new Pose2d[VisionConstants.PoseEstimationLLNames.length];
        public double[] poseTimestampsSeconds = new double[VisionConstants.PoseEstimationLLNames.length];
        public double[] poseLatencies = new double[VisionConstants.PoseEstimationLLNames.length];
        public double[] poseTagDistances = new double[VisionConstants.PoseEstimationLLNames.length];
        public int[] poseTagCounts = new int[VisionConstants.PoseEstimationLLNames.length];
        public int[] nearestTags = new int[VisionConstants.PoseEstimationLLNames.length];

        public int turretCameraAveragePixelValue;
        public boolean objectDetected;
        public double detectedObjectTx;
        public double detectedObjectTy;
        public int detectedObjectLabel;

        public double detectedObjectBB_Corner1x;
        public double detectedObjectBB_Corner1y;
        public double detectedObjectBB_Corner2x;
        public double detectedObjectBB_Corner2y;
        public double detectedObjectBB_Corner3x;
        public double detectedObjectBB_Corner3y;
        public double detectedObjectBB_Corner4x;
        public double detectedObjectBB_Corner4y;
    }

    public default void updateInputs(VisionIOInputs inputs) {}; 

    public default void setRobotOrientation(double yaw, double yawRate) {}

    public default Rotation2d getMT1RotationOf(int index) {
        return Rotation2d.kZero;
    }; 

    public default void setTurretPipeline(TurretPipeline pipeline) {};
}
