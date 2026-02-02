package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand; 
import frc.constants.Controls;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.vision.Vision;
import frc.subsystems.shooter.Shooter;

public class RobotContainer {
    private static RobotContainer instance;
    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private final Vision vision;
    private final OI oi = OI.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    private RobotContainer() {
        // Instantiate vision subsystem first (needed by drive on real robot)
        this.vision = Vision.getInstance();

        setupAuto();
        configureControls();
        neutralizeSubsystems();

        // Register vision subsystem with CommandScheduler so its periodic() method runs
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().registerSubsystem(vision);
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private void setupAuto() {
        //setup auto named commands
    }

    public void teleopEnable() {
        neutralizeSubsystems();
    }

    public void simulationInit() {
        drive.resetPose(new Pose2d(2.0, 4.0, new Rotation2d()));
    }

    private void configureControls() {
        drive.setDefaultCommand(drive.applyRequest(() -> {
            double[] linearInput = oi.getXY();
            double x = linearInput[1], y = linearInput[0];
            double rotation = oi.getRotation();
            return drive.driveOpenLoopFieldCentricRequest.withVelocityX(-x * Controls.MaxDriveMeterS)
                .withVelocityY(-y * Controls.MaxDriveMeterS).withRotationalRate(-rotation * Controls.MaxAngularRadS);
        }).withName("Teleop Default"));
        
        oi.binds.get(OI.Bind.getShooterSpeed).onTrue(new InstantCommand(() -> shooter.getSpeedToHubForPose(drive.getPose()))); 
        oi.binds.get(OI.Bind.getShooterSpeedWithVelocities).onTrue(new InstantCommand(() -> shooter.getSpeedToHubForPoseAndVelocities(drive.getPose()))); 
    }

    public void periodic() {
    }

    public Command getAutonomousCommand() {
        //return chosen autonomous command
        return null; 
    }

    public void neutralizeSubsystems() {
        //neutralize subsystems
    }

    // so vision can get robot pose
    public CommandSwerveDrivetrain getDrive() {
        return drive;
    }

    public Vision getVision() {
        return vision;
    }
}