package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.constants.Controls;
import frc.constants.Controls.GulikitButtons;
import frc.robot.generated.TunerConstants;
//import frc.subsystems.climb.Climber;
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
    //private final Climber climber = Climber.getInstance();
    private final Turret turret = Turret.getInstance(); 
    private boolean inShootingMode = false; 
    private SendableChooser<Command> autoChooser;
    public boolean isRunningFlywheel = false;
    public TalonFX motor = new TalonFX(40);

    private RobotContainer() {
        // Instantiate vision subsystem first (needed by drive on real robot)
        this.vision = Vision.getInstance();
        this.drive = CommandSwerveDrivetrain.getInstance();

        NamedCommands.registerCommand("ShootPathplanner", new InstantCommand(() -> shooter.startShooting()).andThen(new WaitCommand(5)).andThen(new InstantCommand(() -> shooter.stopShooting())));
        NamedCommands.registerCommand("TurnOnFlywheels", new InstantCommand(() -> {isRunningFlywheel = true; shooter.setGoalSpeed(30);}));
        NamedCommands.registerCommand("TurnOnFlywheels", new InstantCommand(() -> isRunningFlywheel = false));

        SmartDashboard.putData("Reset Encoder to 0", new InstantCommand(() -> this.turret.resetEncoder()));
        // these also work while disabled so the pose can be fixed before a match
        SmartDashboard.putData("Reset Position to M2", Commands.runOnce(this::resetTranslationFromVision).ignoringDisable(true));
        SmartDashboard.putData("Reset Rotation to M1", Commands.runOnce(this::resetRotationFromVision).ignoringDisable(true));

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
        autoChooser = AutoBuilder.buildAutoChooser("LoadRight");
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
            double[] linearInput = oi.getXY(false);
            double x = linearInput[1], y = linearInput[0];
            double rotation = oi.getRotation(false);
            return drive.driveOpenLoopFieldCentricRequest.withVelocityX(x * Controls.MaxDriveMeterS)
                .withVelocityY(y * Controls.MaxDriveMeterS).withRotationalRate(-rotation * Controls.MaxAngularRadS);
        }).withName("Teleop Default"));
        
        // oi.binds.get(OI.Bind.enterShootingMode).onTrue(new InstantCommand(() -> inShootingMode = !inShootingMode)); 
        // oi.binds.get(OI.Bind.startShooting).onTrue(new InstantCommand(() -> shooter.startShooting())).onFalse(new InstantCommand(() -> shooter.stopShooting()));
        oi.binds.get(OI.Bind.manualTurretLeft).onTrue(new InstantCommand(() -> {inShootingMode=false; System.out.println("up 0.5!");})).whileTrue(Commands.run(() -> turret.setControllerChange(0.05)));  
        oi.binds.get(OI.Bind.manualTurretRight).onTrue(new InstantCommand(() -> {inShootingMode=false; System.out.println("down 0.5!");})).whileTrue(Commands.run(() -> turret.setControllerChange(-0.05))); 
        oi.binds.get(OI.Bind.manualSpeedUp).onTrue(new InstantCommand(() -> inShootingMode=false)).whileTrue(Commands.run(() -> shooter.setGoalSpeed(shooter.getGoalSpeed()+0.5)));  
        oi.binds.get(OI.Bind.manualSpeedDown).onTrue(new InstantCommand(() -> inShootingMode=false)).whileTrue(Commands.run(() -> shooter.setGoalSpeed(shooter.getGoalSpeed()-0.5))); 
        oi.binds.get(OI.Bind.manualTurretSwitch).onTrue(new InstantCommand(() -> System.out.println("TESTING SWITCH"))).whileTrue(new InstantCommand(() -> turret.IpswitchPitchSwitch()));
        // oi.binds.get(OI.Bind.setHighGoal).whileTrue(new InstantCommand(() -> climber.setClimberChange(0.6))); 
        // oi.binds.get(OI.Bind.setLowGoal).whileTrue(new InstantCommand(() -> climber.setClimberChange(-0.6)));
        oi.binds.get(OI.Bind.intake).onTrue(new InstantCommand(() -> {System.out.println("SWITCHING"); intake.switchSpinForComp();})); 
        oi.binds.get(OI.Bind.intakeRun).onTrue(new InstantCommand(() -> {System.out.println("RUNNING ITNAKE"); intake.toggleIntake();})); 
        oi.binds.get(OI.Bind.flywheel).onTrue(new InstantCommand(() -> isRunningFlywheel = !isRunningFlywheel)); 
        // driver fix for flipped/rotated field-centric driving: makes the way the robot is facing right now "forward"
        oi.binds.get(OI.Bind.reZeroFieldCentric).onTrue(drive.runOnce(() -> {System.out.println("Re-zeroing field-centric forward"); drive.seedFieldCentric();}).ignoringDisable(true));
        //oi.binds.get(OI.Bind.climberVoltageToZero).onTrue(new InstantCommand(() -> climber.setVoltage(0))); 
    }

    public void periodic() {
        //Logger.recordOutput("Turret/Voltage", turret.getMotorVoltage());
        // Logger.recordOutput("Intake/position", intake.intakePush.getPosition().getValueAsDouble());
        Logger.recordOutput("Shooter/Voltage", shooter.getMotor1Voltage());
        Logger.recordOutput("Shooter/Voltage2", shooter.getMotor1Voltage());
        Logger.recordOutput("isFlywheelRunning", isRunningFlywheel);
        Logger.recordOutput("Drive/backLeftOffset", motor.getPosition().getValueAsDouble());
    }

    /** Moves the robot's X/Y to where vision sees it, keeping the current heading so field-centric driving doesn't change */
    private void resetTranslationFromVision() {
        Translation2d visionTranslation = vision.getCurrentAverageTranslation();
        if (visionTranslation == null) {
            DriverStation.reportWarning("Reset Position to M2: no tags visible, pose not changed", false);
            return;
        }
        drive.resetTranslation(visionTranslation);
    }

    /** Sets the robot's heading from megatag 1. Only does anything if a camera sees 2+ tags and the cameras agree */
    private void resetRotationFromVision() {
        Rotation2d visionRotation = vision.getCurrentAverageRotation();
        if (visionRotation == null) {
            DriverStation.reportWarning("Reset Rotation to M1: no camera sees 2+ tags (or cameras disagree), heading not changed", false);
            return;
        }
        System.out.println("Resetting rotation to " + visionRotation.getDegrees() + " deg from megatag 1");
        drive.resetRotation(visionRotation);
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
}