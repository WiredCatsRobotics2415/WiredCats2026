package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.vision.Vision;

public class RobotContainer {
    private static RobotContainer instance;
    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private final Vision vision;
    private final OI oi = OI.getInstance();

    private RobotContainer() {
        // Instantiate vision subsystem first (needed by drive on real robot)
        this.vision = Vision.getInstance();

        setupAuto();
        configureControls();
        neutralizeSubsystems();
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