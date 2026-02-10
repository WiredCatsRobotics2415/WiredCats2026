package frc.robot;

import java.util.Optional;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand; 
import frc.constants.Controls;
import frc.constants.Measurements;
import frc.subsystems.climb.Climber;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.vision.Vision;
import frc.subsystems.shooter.Shooter;
import frc.subsystems.turret.Turret; 


public class RobotContainer {
    private static RobotContainer instance;
    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private final Vision vision;
    private final OI oi = OI.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Turret turret = Turret.getInstance(); 
    private boolean inShootingMode = false; 

    private RobotContainer() {
        // Instantiate vision subsystem first (needed by drive on real robot)
        this.vision = Vision.getInstance();

        setupAuto();
        configureControls();
        neutralizeSubsystems();

        // Register subsystems with CommandScheduler so their periodic() methods run
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().registerSubsystem(vision);
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().registerSubsystem(shooter);
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().registerSubsystem(turret);
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
        //Measurements.AprilTagLocations.addAllPoses();
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
        
        oi.binds.get(OI.Bind.enterShootingMode).onTrue(new InstantCommand(() -> inShootingMode = !inShootingMode)); 
        oi.binds.get(OI.Bind.shoot).onTrue(new InstantCommand(() -> shooter.startShooting())); 
        oi.binds.get(OI.Bind.setHighGoal).onTrue(climber.SetVolt(3.0)); 
        oi.binds.get(OI.Bind.setLowGoal).onTrue(climber.SetVolt(0.0)); 
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
    
    public boolean isInShootingMode() {
        return inShootingMode;
    }

    // so vision can get robot pose
    public CommandSwerveDrivetrain getDrive() {
        return drive;
    }

    public Vision getVision() {
        return vision;
    }

    public Pose2d getHubPosition() //If no alliance returns blue hub location
    {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Blue ? Measurements.HubLocation : flipPose(Measurements.HubLocation);
    }
    static Pose2d flipPose(Pose2d pose) //Helper method
    {
        return new Pose2d(Measurements.ApriltagFieldLayout.getFieldLength() - pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
}