package frc.subsystems.vision;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.Measurements;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.Measurements.LimelightSpecs;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.utils.LimelightHelpers.PoseEstimate;

import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class VisionIOSim implements VisionIO {
    private VisionSystemSim visionSystemSim; 
    private PhotonCameraSim frontLeftSimCam; 
    private PhotonCameraSim frontRightSimCam; 
    private PhotonCameraSim backSimCam; 

    private PhotonCamera frontLeftCam; 
    private PhotonCamera frontRightCam; 
    private PhotonCamera backCam; 

    private PhotonCamera[] poseEstimationCameras; 
    private PhotonPoseEstimator[] photonEstimators; 

    public VisionIOSim() {
        visionSystemSim = new VisionSystemSim("main"); 
        visionSystemSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded));

        SimCameraProperties limelightCameraProps = new SimCameraProperties(); 
        // settings from 2025
        limelightCameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(LimelightSpecs.ThreeGDiagnolFOV));
        limelightCameraProps.setCalibError(0.25, 0.08);
        limelightCameraProps.setFPS(20);
        limelightCameraProps.setAvgLatencyMs(60);
        limelightCameraProps.setLatencyStdDevMs(7.5);

        frontLeftSimCam = new PhotonCameraSim(new PhotonCamera("Front Left LL"), limelightCameraProps);
        frontLeftCam = frontLeftSimCam.getCamera();
        frontRightSimCam = new PhotonCameraSim(new PhotonCamera("Front Right LL"), limelightCameraProps);
        frontRightCam = frontRightSimCam.getCamera();
        backSimCam = new PhotonCameraSim(new PhotonCamera("Back LL"), limelightCameraProps);
        backCam = backSimCam.getCamera();

        visionSystemSim.addCamera(frontLeftSimCam, RobotMeasurements.FrontLeftCamera);
        visionSystemSim.addCamera(frontRightSimCam, RobotMeasurements.FrontRightCamera);
        visionSystemSim.addCamera(backSimCam, RobotMeasurements.BackCamera);

        photonEstimators = new PhotonPoseEstimator[] {
            new PhotonPoseEstimator(Measurements.ApriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, RobotMeasurements.FrontLeftCamera), 
            new PhotonPoseEstimator(Measurements.ApriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, RobotMeasurements.FrontRightCamera),
            new PhotonPoseEstimator(Measurements.ApriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, RobotMeasurements.BackCamera),
        }; 
        
        for (PhotonPoseEstimator estimator : photonEstimators) {
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); 
        }

        poseEstimationCameras = new PhotonCamera[] {frontLeftCam, frontRightCam, backCam}; 
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d currentRobotPose = RobotContainer.getInstance().getDrive().getPose();
        visionSystemSim.update(currentRobotPose);

        // Log the pose being used for vision simulation
        Logger.recordOutput("/Vision/SimulatorInputPose", currentRobotPose); 

        // creates new pose metadata variables storing arrays w/ one element per limelight
        inputs.poseEstimates = new Pose2d[poseEstimationCameras.length];
        inputs.poseLatencies = new double[poseEstimationCameras.length];
        inputs.poseTimestampsSeconds = new double[poseEstimationCameras.length];
        inputs.poseTagCounts = new int[poseEstimationCameras.length];
        inputs.poseTagDistances = new double[poseEstimationCameras.length];
        inputs.nearestTags = new int[poseEstimationCameras.length]; 

        for (int i = 0; i < poseEstimationCameras.length; i++) {
            List<PhotonPipelineResult> pipelineResults = poseEstimationCameras[i].getAllUnreadResults(); 
            if (pipelineResults.size() > 0) {
                PhotonPipelineResult pipelineResult = pipelineResults.get(pipelineResults.size() - 1); 
                EstimatedRobotPose estimatedPose = photonEstimators[i].update(pipelineResult).orElse(null);
                if (estimatedPose != null) {
                    inputs.poseEstimates[i] = estimatedPose.estimatedPose.toPose2d();
                    inputs.poseLatencies[i] = pipelineResult.metadata.getLatencyMillis();
                    inputs.poseTimestampsSeconds[i] = estimatedPose.timestampSeconds;
                    inputs.poseTagCounts[i] = estimatedPose.targetsUsed.size();

                    double smallestDistance = Double.MAX_VALUE;
                    int closestApriltagId = -1;
                    for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                        if (distance < smallestDistance) {
                            smallestDistance = distance;
                            closestApriltagId = target.fiducialId;
                        }
                    }
                    inputs.poseTagDistances[i] = smallestDistance;
                    inputs.nearestTags[i] = closestApriltagId;
                } else {
                    setInputsWhenNoTagDetected(inputs, i); 
                }
            } else {
                setInputsWhenNoTagDetected(inputs, i); 
            }
        }
    }

    private void setInputsWhenNoTagDetected(VisionIOInputs inputs, int index) {
        PoseEstimate estimate = PoseEstimate.zero; 
        inputs.poseEstimates[index] = estimate.pose; 
        inputs.poseLatencies[index] = estimate.latency; 
        inputs.poseTimestampsSeconds[index] = estimate.timestampSeconds; 
        inputs.poseTagCounts[index] = estimate.tagCount;
        inputs.poseTagDistances[index] = Double.MAX_VALUE; 
        inputs.nearestTags[index] = -1; 
    }
}