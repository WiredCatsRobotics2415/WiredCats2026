package frc.constants;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import org.ironmaple.simulation.drivesims.COTS;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static class AprilTagLocations {

    // Map of tag ID -> Pose3d
    public static final Map<Integer, Pose3d> TAGS = Map.ofEntries(
      Map.entry(1,  pose(11.8779798, 7.4247756, 0.889,    6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(2,  pose(11.9154194, 4.638039999999999, 1.12395, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476)),
      Map.entry(3,  pose(11.3118646, 4.3902376, 1.12395,  6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(4,  pose(11.3118646, 4.0346376, 1.12395,  6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(5,  pose(11.9154194, 3.4312351999999997, 1.12395, -0.7071067811865475, -0.0, 0.0, 0.7071067811865476)),
      Map.entry(6,  pose(11.8779798, 0.6444996, 0.889,    6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(7,  pose(11.9528844, 0.6444996, 0.889,    1.0, 0.0, 0.0, 0.0)),
      Map.entry(8,  pose(12.2710194, 3.4312351999999997, 1.12395, -0.7071067811865475, -0.0, 0.0, 0.7071067811865476)),
      Map.entry(9,  pose(12.519177399999998, 3.6790375999999996, 1.12395, 1.0, 0.0, 0.0, 0.0)),
      Map.entry(10, pose(12.519177399999998, 4.0346376, 1.12395, 1.0, 0.0, 0.0, 0.0)),
      Map.entry(11, pose(12.2710194, 4.638039999999999, 1.12395, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476)),
      Map.entry(12, pose(11.9528844, 7.4247756, 0.889,    1.0, 0.0, 0.0, 0.0)),
      Map.entry(13, pose(16.5333172, 7.4033126, 0.55245,  6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(14, pose(16.5333172, 6.9715126, 0.55245,  6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(15, pose(16.5329616, 4.3235626, 0.55245,  6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(16, pose(16.5329616, 3.8917626, 0.55245,  6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(17, pose(4.6630844, 0.6444996, 0.889,     1.0, 0.0, 0.0, 0.0)),
      Map.entry(18, pose(4.6256194, 3.4312351999999997, 1.12395, -0.7071067811865475, -0.0, 0.0, 0.7071067811865476)),
      Map.entry(19, pose(5.229174199999999, 3.6790375999999996, 1.12395, 1.0, 0.0, 0.0, 0.0)),
      Map.entry(20, pose(5.229174199999999, 4.0346376, 1.12395, 1.0, 0.0, 0.0, 0.0)),
      Map.entry(21, pose(4.6256194, 4.638039999999999, 1.12395, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476)),
      Map.entry(22, pose(4.6630844, 7.4247756, 0.889,     1.0, 0.0, 0.0, 0.0)),
      Map.entry(23, pose(4.5881798, 7.4247756, 0.889,     6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(24, pose(4.2700194, 4.638039999999999, 1.12395, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476)),
      Map.entry(25, pose(4.0218614, 4.3902376, 1.12395,   6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(26, pose(4.0218614, 4.0346376, 1.12395,   6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(27, pose(4.2700194, 3.4312351999999997, 1.12395, -0.7071067811865475, -0.0, 0.0, 0.7071067811865476)),
      Map.entry(28, pose(4.5881798, 0.6444996, 0.889,     6.123233995736766e-17, 0.0, 0.0, 1.0)),
      Map.entry(29, pose(0.0077469999999999995, 0.6659626, 0.55245, 1.0, 0.0, 0.0, 0.0)),
      Map.entry(30, pose(0.0077469999999999995, 1.0977626, 0.55245, 1.0, 0.0, 0.0, 0.0)),
      Map.entry(31, pose(0.0080772, 3.7457125999999996, 0.55245, 1.0, 0.0, 0.0, 0.0)),
      Map.entry(32, pose(0.0080772, 4.1775126, 0.55245, 1.0, 0.0, 0.0, 0.0))
  );

  /** Convenience accessor. */
  public static Pose3d AprilTagePose(int id) {
    return TAGS.get(id);
  }

  /** Helper: build Pose3d from translation + quaternion (w, x, y, z). */
  private static Pose3d pose(double x, double y, double z,
                             double qw, double qx, double qy, double qz) {
    return new Pose3d(
        new Translation3d(x, y, z),
        new Rotation3d(new Quaternion(qw, qx, qy, qz))
    );
  }

  public static void addAllPoses() {
    for (int i = 0; i < 32; i++) {
        String name = "/Apriltags/tag " + i + " pose";
        Logger.recordOutput(name, AprilTagePose(i));
    }
  }

  //FRONT RIGHT: x=0.266m, y=0.280m, z=0.497, angle=55 degrees from front
  //FRONT LEFT: x=-0.266m, y=0.280m, z=0.497, angle=55 degrees from front
  //BACK: x=-0.293m, y=0.288m, z=0.497m, angle=7.636 facing inwards

  public static Translation3d getLimelightOffsets(int id) {
    switch (id) {
        case 0: 
            return new Translation3d(0.1,-0.1,0.5);
        case 1:
            return new Translation3d(0.1,0.1,0.5);
        case 2:
            return new Translation3d(-0.1,0,0.5);
        default:
            return new Translation3d(0,0,0);
    }
  }


    }

    public static final Pose2d HubLocation = new Pose2d(181.56 / 39.3701, 158.84 / 39.3701, Rotation2d.fromDegrees(0.0)); // inches to meters

    public static final double ShooterAngleLow = 15; 
    public static final double ShooterAngleHigh = 45; 

    public static final double ShooterHubRegionOne = 6;

    public static final double TurretAngleOffset = 30; 
}