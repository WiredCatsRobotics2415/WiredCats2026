package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.constants.Controls;
import frc.robot.generated.TunerConstants;
import frc.subsystems.climb.Climber;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.intake.Intake;
import frc.subsystems.vision.Vision;
import frc.subsystems.shooter.Shooter;
import frc.subsystems.turret.Turret; 

public class RobotContainer {
    private static RobotContainer instance;
    private CommandSwerveDrivetrain drive;
    private final Vision vision;
    private final OI oi = OI.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Turret turret = Turret.getInstance(); 
    private boolean inShootingMode = false; 
    private SendableChooser<Command> autoChooser;
    public boolean isRunningFlywheel = false;

    private RobotContainer() {
        // Instantiate vision subsystem first (needed by drive on real robot)
        this.vision = Vision.getInstance();
        this.drive = CommandSwerveDrivetrain.getInstance();

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
        autoChooser = AutoBuilder.buildAutoChooser("AUTO1");
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
            return drive.driveOpenLoopFieldCentricRequest.withVelocityX(x * Controls.MaxDriveMeterS)
                .withVelocityY(y * Controls.MaxDriveMeterS).withRotationalRate(rotation * Controls.MaxAngularRadS);
        }).withName("Teleop Default"));
        
        // oi.binds.get(OI.Bind.enterShootingMode).onTrue(new InstantCommand(() -> inShootingMode = !inShootingMode)); 
        oi.binds.get(OI.Bind.startShooting).onTrue(new InstantCommand(() -> shooter.startShooting())).onFalse(new InstantCommand(() -> shooter.stopShooting()));
        oi.binds.get(OI.Bind.manualTurretLeft).onTrue(new InstantCommand(() -> inShootingMode=false)).whileTrue(Commands.run(() -> turret.setControllerChange(0.05)));  
        oi.binds.get(OI.Bind.manualTurretRight).onTrue(new InstantCommand(() -> inShootingMode=false)).whileTrue(Commands.run(() -> turret.setControllerChange(-0.05))); 
        oi.binds.get(OI.Bind.manualSpeedUp).onTrue(new InstantCommand(() -> inShootingMode=false)).whileTrue(Commands.run(() -> shooter.setGoalSpeed(shooter.getGoalSpeed()+0.5)));  
        oi.binds.get(OI.Bind.manualSpeedDown).onTrue(new InstantCommand(() -> inShootingMode=false)).whileTrue(Commands.run(() -> shooter.setGoalSpeed(shooter.getGoalSpeed()-0.5))); 
        // oi.binds.get(OI.Bind.manualTurretLeft).whileTrue(Commands.run(() -> turret.movePosChange(0.3)));  
        // oi.binds.get(OI.Bind.manualTurretRight).whileTrue(Commands.run(() -> turret.movePosChange(-0.3))); 
        oi.binds.get(OI.Bind.manualTurretUp).onTrue(new InstantCommand(() -> inShootingMode=false)).whileTrue(Commands.run(() -> turret.setPitchAngle(turret.getPitchAngle() - 1)));  
        oi.binds.get(OI.Bind.manualTurretDown).onTrue(new InstantCommand(() -> inShootingMode=false)).whileTrue(Commands.run(() -> turret.setPitchAngle(turret.getPitchAngle() + 1)));
        oi.binds.get(OI.Bind.manualTurretSwitch).onTrue(new InstantCommand(() -> inShootingMode=false)).whileTrue(new InstantCommand(() -> turret.IpswitchPitchSwitch()));
        oi.binds.get(OI.Bind.setHighGoal).whileTrue(climber.SetVolt(3.0));
        oi.binds.get(OI.Bind.setLowGoal).onTrue(climber.SetVolt(0.0)); 
        oi.binds.get(OI.Bind.climbHighGoal).onTrue(climber.SetVolt(6)); 
        oi.binds.get(OI.Bind.climbLowGoal).onTrue(climber.SetVolt(0)); 
        oi.binds.get(OI.Bind.climbZero).onTrue(climber.SetVolt(3)); 
        oi.binds.get(OI.Bind.intake).onTrue(new InstantCommand(() -> intake.switchSpinForComp())); 
        oi.binds.get(OI.Bind.flywheel).onTrue(new InstantCommand(() -> isRunningFlywheel = !isRunningFlywheel)); 
    }

    public void periodic() {
        //Logger.recordOutput("Turret/Voltage", turret.getMotorVoltage());
        // Logger.recordOutput("Intake/position", intake.intakePush.getPosition().getValueAsDouble());
        Logger.recordOutput("Shooter/Voltage", shooter.getMotor1Voltage());
        Logger.recordOutput("Shooter/Voltage2", shooter.getMotor1Voltage());
        Logger.recordOutput("isFlywheelRunning", isRunningFlywheel);
    }

    public Command getAutonomousCommand() {
        System.out.println("Selected auto: " + autoChooser.getSelected());
        return autoChooser.getSelected();
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

    public void ShootPathPlanner() //TODO: Hastily written code, hope it works but if robot doesn't shoot during auton this is the issue
    {
        inShootingMode = true;
        Shooter.getInstance().startShooting();
        new WaitCommand(5);
        inShootingMode = false;
        Shooter.getInstance().stopShooting();
    }
}