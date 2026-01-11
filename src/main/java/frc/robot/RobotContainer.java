package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.subsystems.drive.MapleSimSwerve;
import frc.subsystems.drive.SwerveDrive;
import frc.subsystems.drive.SwerveDriveReal;

public class RobotContainer {
    private static RobotContainer instance;
    private final SwerveDrive drive;
    private final OI oi = OI.getInstance();

    private RobotContainer() {
        setupAuto();
        configureControls();
        neutralizeSubsystems();

        //real or simulated drivetrain
        if (Robot.isReal()) {
            this.drive = new SwerveDriveReal(); // Real implementation
            }
        else {
            this.drive = new MapleSimSwerve(); // Simulation implementation
        }
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
}