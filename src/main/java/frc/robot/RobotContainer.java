package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.subsystems.drive.MapleSimSwerve;
import frc.subsystems.drive.SwerveDrive;
import frc.subsystems.drive.SwerveDriveReal;
import frc.subsystems.vision.Vision;

public class RobotContainer {
    private static RobotContainer instance;
    private final SwerveDrive drive;
    private final Vision vision;
    private final OI oi = OI.getInstance();

    private RobotContainer() {
        // Instantiate vision subsystem first (needed by drive on real robot)
        this.vision = Vision.getInstance();

        //real or simulated drivetrain
        if (Robot.isReal()) {
            this.drive = new SwerveDriveReal(vision); // Real implementation with vision
            }
        else {
            this.drive = new MapleSimSwerve(vision); // Simulation implementation with vision
        }

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
        //whenever drive is not being used, it will go back to this default
        drive.setDefaultCommand(
        new RunCommand(() -> {
          double x = oi.getXY()[0] * SwerveDrive.kMaxSpeed;
          double y = oi.getXY()[1] * SwerveDrive.kMaxSpeed;
          double rot = oi.getRotation() * SwerveDrive.kMaxAngularSpeed;

          drive.drive(x, y, rot, true); // fieldRelative true
        }, drive)
    );
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
    public SwerveDrive getDrive() {
        return drive;
    }

    // Returns the fused pose estimate (odometry + vision)
    public Pose2d getGeneralPose() {
        return drive.getPose();
    }

    public Vision getVision() {
        return vision;
    }
}