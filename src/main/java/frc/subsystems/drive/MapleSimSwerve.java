package frc.subsystems.drive;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.subsystems.vision.Vision;
import frc.utils.LimelightHelpers.PoseEstimate;

public class MapleSimSwerve implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Vision m_vision;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private SwerveModulePosition[] m_lastModulePositions;

    public MapleSimSwerve(Vision vision) {
        this.m_vision = vision;
        System.out.println("RUNNING MAPLE SIM");

        // For your own code, please configure your drivetrain properly according to the documentation
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();

        // Creating the SelfControlledSwerveDriveSimulation instance
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d())));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // Initialize pose estimator for vision fusion
        // We use IronMaple's odometry as the base and fuse vision on top
        m_lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        // Create minimal kinematics for pose estimator (matching real robot dimensions)
        SwerveDriveKinematics tempKinematics = new SwerveDriveKinematics(
            new Translation2d(0.381, 0.381),   // Front Left
            new Translation2d(0.381, -0.381),  // Front Right
            new Translation2d(-0.381, 0.381),  // Back Left
            new Translation2d(-0.381, -0.381)  // Back Right
        );

        m_poseEstimator = new SwerveDrivePoseEstimator(
            tempKinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),    // Trust IronMaple odometry
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))      // Vision std devs
        );
    }

    /** Called every 20ms by the scheduler */
    @Override
    public void periodic() {
        // Update IronMaple physics simulation
        simulatedDrive.periodic();

        // Update vision fusion
        updateOdometryAndVision();

        // Log all pose estimates for analysis
        Pose2d groundTruth = getGroundTruthPose();
        Pose2d fusedPose = getPose();

        Logger.recordOutput("/Drive/GroundTruth", groundTruth);
        Logger.recordOutput("/Drive/IronMapleOdometry", simulatedDrive.getOdometryEstimatedPose());
        Logger.recordOutput("/Drive/FusedPose", fusedPose);

        // Calculate and log drift from ground truth
        Pose2d drift = fusedPose.relativeTo(groundTruth);
        Logger.recordOutput("/Drive/DriftFromGroundTruth", drift);
    }

    /** Command-friendly drive method (expects m/s and rad/s). */
    @Override
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        this.simulatedDrive.runChassisSpeeds(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                new Translation2d(),
                fieldRelative,
                true);
    }

    @Override
    public void stop() {
        drive(0, 0, 0, false);
    }

    @Override
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    @Override
    public void resetPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);

        // Reset our pose estimator too
        SwerveModulePosition[] currentPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
        m_poseEstimator.resetPosition(pose.getRotation(), currentPositions, pose);
        m_lastModulePositions = currentPositions;
    }

    /** Returns ground truth pose from simulation (for debugging and vision camera sim) */
    @Override
    public Pose2d getGroundTruthPose() {
        return simulatedDrive.getActualPoseInSimulationWorld();
    }

    /** Updates odometry with IronMaple data and fuses vision measurements */
    private void updateOdometryAndVision() {
        // 1. Get IronMaple's odometry estimate (this is the "wheel odometry" for us)
        Pose2d ironMapleOdom = simulatedDrive.getOdometryEstimatedPose();
        Pose2d lastEstimate = m_poseEstimator.getEstimatedPosition();

        // 2. Calculate the translation distance since last update
        Translation2d currentTranslation = ironMapleOdom.getTranslation();
        Translation2d lastTranslation = lastEstimate.getTranslation();
        double distanceDelta = currentTranslation.getDistance(lastTranslation);

        // 3. Update module positions (all modules travel approximately the same distance)
        // This is a simplification - IronMaple handles the actual kinematics internally
        // IMPORTANT: Module angles should be steering angles, NOT robot heading!
        // Using zero angles assumes all modules point forward relative to robot
        SwerveModulePosition[] newModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(m_lastModulePositions[0].distanceMeters + distanceDelta, new Rotation2d()),
            new SwerveModulePosition(m_lastModulePositions[1].distanceMeters + distanceDelta, new Rotation2d()),
            new SwerveModulePosition(m_lastModulePositions[2].distanceMeters + distanceDelta, new Rotation2d()),
            new SwerveModulePosition(m_lastModulePositions[3].distanceMeters + distanceDelta, new Rotation2d())
        };

        // 4. Properly update pose estimator using update() method (enables Kalman filtering)
        m_poseEstimator.update(ironMapleOdom.getRotation(), newModulePositions);
        m_lastModulePositions = newModulePositions;

        // 5. Send orientation to vision subsystem (required for MegaTag2)
        m_vision.sendOrientation(
            getPose().getRotation().getDegrees(),
            0  // No gyro rate in sim (always stable)
        );

        // 6. Get vision estimates from all 3 simulated cameras
        PoseEstimate[] visionEstimates = m_vision.getPoseEstimates();

        // 7. Fuse vision measurements (same as SwerveDriveReal)
        boolean rejectUpdate = false; // Sim is always stable (no spinning)

        if (!rejectUpdate) {
            for (int i = 0; i < visionEstimates.length; i++) {
                PoseEstimate estimate = visionEstimates[i];

                // Only add measurement if we detected at least one AprilTag
                if (estimate.tagCount > 0) {
                    // Set standard deviations (trust vision for position, ignore rotation)
                    // X/Y: 0.7m uncertainty, Rotation: effectively infinite (trust gyro instead)
                    m_poseEstimator.setVisionMeasurementStdDevs(
                        VecBuilder.fill(0.7, 0.7, 9999999)
                    );

                    // Add the vision measurement with its timestamp
                    m_poseEstimator.addVisionMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds
                    );
                    final String this_name = "/Drive/Limelight" + i;
                    Logger.recordOutput(this_name, estimate.pose);
                }
            }
        }
    }
}