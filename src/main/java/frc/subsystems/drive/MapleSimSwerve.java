package frc.subsystems.drive;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MapleSimSwerve implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;

    public MapleSimSwerve() {
        // For your own code, please configure your drivetrain properly according to the documentation
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();

        // Creating the SelfControlledSwerveDriveSimulation instance
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d())));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // A field2d widget for debugging
        field2d = new Field2d();
        SmartDashboard.putData("simulation field", field2d);
    }

    /** Called every 20ms by the scheduler */
    @Override
    public void periodic() {
        // update the odometry of the SimplifedSwerveSimulation instance
        simulatedDrive.periodic();

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());
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
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void resetPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }
}