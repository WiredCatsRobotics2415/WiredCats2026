package frc.constants;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.drivesims.COTS;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.constants.Subsystems.DriveConstants;

public class Measurements {
    
    // not sure if the new field is out yet
    public static final AprilTagFieldLayout ApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); 

    public class RobotMeasurements {
        // Front of the robot: coral scoring side
        public static final Distance BumperLength = Inches.of(3.204);
        public static final Distance BumperToBumper = Inches.of(36);

        public static final Distance CenterToFrameRadius = Inches.of(21.313);
        public static final Distance CenterToFramePerpendicular = Inches.of(15.401);
        public static final Distance DriveTrainRadius = Inches.of(18.432785);
        public static final Distance DriveTrainTrackWidth = Inches.of(24.625);

        public static final Angle ElevatorTilt = Degrees.of(3.7); // Towards the front

        public static final Mass RobotWeight = Pounds.of(131); // approx, with bumpers and battery
        public static final MomentOfInertia RobotMOI = KilogramSquareMeters.of(RobotWeight.in(Kilograms) *
            (DriveTrainTrackWidth.in(Meters) / 2) * (DriveConstants.HeadingKA / TunerConstants.driveGains.kA));
        public static final ModuleConfig SwerveModuleConfig = new ModuleConfig(TunerConstants.kWheelRadius,
            TunerConstants.kSpeedAt12Volts, COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof, // TODO: find this with slip current characerization
            DCMotor.getKrakenX60Foc(1), TunerConstants.kSlipCurrent, 1);

        public static final RobotConfig PPRobotConfig = new RobotConfig(RobotWeight, RobotMOI, SwerveModuleConfig,
            new Translation2d[] { new Translation2d(TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos),
                new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos),
                new Translation2d(TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos),
                new Translation2d(TunerConstants.kBackRightYPos, TunerConstants.kBackRightYPos) });
        static {
            System.out.println("PP Robot Config: ");
            System.out.println("    Mass (KG): " + RobotWeight.in(Kilograms));
            System.out.println("    Moi: " + RobotMOI.in(KilogramSquareMeters));
            System.out.println("    Wheel Radius (M): " + TunerConstants.kWheelRadius.in(Meters));
            System.out.println("    Drive Gearing: " + TunerConstants.kDriveGearRatio);
            System.out.println("    True Max Drive Speed: " + TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
            System.out.println("    Wheel COF: " + SwerveModuleConfig.wheelCOF);
            System.out.println("    Drive Motor: " + SwerveModuleConfig.driveMotor);
            System.out.println("    Drive Current Limit: " + TunerConstants.kSlipCurrent.in(Amps));
            System.out.println("    Module Offsets (FL, FR, BL, BR): ");
            for (Translation2d location : PPRobotConfig.moduleLocations) {
                System.out.println("        " + location.toString());
            }
        }

        public static final Transform3d FrontLeftCamera = new Transform3d(Inches.of(2.644), Inches.of(-11.784437),
            Inches.of(26.531608), new Rotation3d(Degrees.of(0), Degrees.of(3.7 + 16.5), Degrees.of(-20)));
        public static final Transform3d FrontRightCamera = new Transform3d(Inches.of(2.644), Inches.of(11.784437),
            Inches.of(26.531608), new Rotation3d(Degrees.of(0), Degrees.of(3.7 + 16.5), Degrees.of(20)));
        public static final Transform3d BackCamera = new Transform3d(Inches.of(-7.578), Inches.of(10.052),
            Inches.of(27.982014), new Rotation3d(Degrees.of(0), Degrees.of(3.7 - 32), Degrees.of(-190)));
        public static final Transform3d[] PECameraTransforms = new Transform3d[] { FrontLeftCamera, FrontRightCamera,
            BackCamera };

        public static final Transform2d EECamOnGround = new Transform2d(Inches.of(-11.972173), Inches.of(0),
            Rotation2d.fromDegrees(180));
        public static final double EECamHeightOffGround = 13.82;
        public static final double EECamForward = 1.284;
    }

    // from last year
    public static class LimelightSpecs {
        public static final double ThreeGFOV = 82;
        public static final double ThreeGVerticalFOV = 56.2;
        public static final double ThreeGDiagnolFOV = 91.144;

        public static final double TwoPlusHorizontalFOV = 62.5;
        public static final double TwoPlusVerticalFOV = 48.9;
        public static final double TwoPlusDiagnolFOV = 2 *
            Math.atan(Math.sqrt(Math.pow(Math.tan(Units.degreesToRadians(TwoPlusHorizontalFOV) / 2), 2) +
                Math.pow(Math.tan(Units.degreesToRadians(TwoPlusVerticalFOV) / 2), 2)));

        public static final double TwoPlusMaxObjectDetectionDistance = 6 * 12;
    }

    public static final Pose2d HubLocation = new Pose2d(5.0, 10.0, Rotation2d.fromDegrees(45.0)); 

    public static final double ShooterRPMOne = 20; 
    public static final double ShooterRPMTwo = 10; 
}