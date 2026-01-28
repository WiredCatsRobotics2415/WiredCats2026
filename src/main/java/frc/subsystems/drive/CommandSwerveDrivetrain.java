package frc.subsystems.drive;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.constants.Controls;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.RuntimeConstants;
import frc.constants.TunerConstants;
import frc.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.subsystems.vision.Vision;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.002;
    private Notifier simNotifier = null;

    private Pose2d robotPose = new Pose2d();

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.FieldCentric driveOpenLoopFieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Controls.MaxDriveMeterS * 0.05).withRotationalDeadband(Controls.MaxAngularRadS * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    public final SwerveRequest.RobotCentric driveOpenLoopRobotCentricRequest = new SwerveRequest.RobotCentric()
        .withDeadband(Controls.MaxDriveMeterS * 0.05).withRotationalDeadband(Controls.MaxAngularRadS * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.FieldCentricFacingAngle driveToPositionFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private Vision vision = Vision.getInstance();
    private RealPose realPose;
    private SimulatePose simulatePose;

    private static CommandSwerveDrivetrain instance;

    private CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
            // Start robot out farther in field so collisions don't apply
            resetPose(new Pose2d(3, 3, Rotation2d.kZero));
            simulatePose = new SimulatePose(this, vision);
        } else {
            realPose = new RealPose(this, vision);
        }
        configureAutoBuilder();


        if (RuntimeConstants.TuningMode) {
            //do stuff for tuning, tbd
            };

    }

    public static CommandSwerveDrivetrain getInstance() {
        if (instance == null) instance = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        return instance;
    }

    private void configureAutoBuilder() {
        //do stuff for auto, tbd
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Pose2d getPose() {
       return robotPose;
    }

    @Override
    public void periodic() {
        // Update odometry and vision based on simulation vs real robot
        if (Utils.isSimulation()) {
            simulatePose.updateOdometryAndVision();
        } else {
            realPose.updateOdometryAndVision();
        }

        // Update the robot pose from the drivetrain state
        robotPose = getState().Pose;
    }

    @Getter private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(Seconds.of(kSimLoopPeriod),
            RobotMeasurements.RobotWeight, RobotMeasurements.BumperToBumper, RobotMeasurements.BumperToBumper,
            DCMotor.getKrakenX60Foc(1), DCMotor.getFalcon500(1), RobotMeasurements.SwerveModuleConfig.wheelCOF,
            getModuleLocations(), getPigeon2(), getModules(), TunerConstants.FrontLeft, TunerConstants.FrontRight,
            TunerConstants.BackLeft, TunerConstants.BackRight);
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.1); // wait for simulation to update
        }
        super.resetPose(pose);
    }
}