package frc.constants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

public class Measurements {
    
    // not sure if the new field is out yet
    public static final AprilTagFieldLayout ApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); 

    public static class RobotMeasurements {
        // last year's values
        public static final Distance CenterToFrameRadius = Inches.of(21.313);
        public static final Transform3d FrontLeftCamera = new Transform3d(Inches.of(2.644), Inches.of(-11.784437),
            Inches.of(26.531608), new Rotation3d(Degrees.of(0), Degrees.of(3.7 + 16.5), Degrees.of(-20)));
        public static final Transform3d FrontRightCamera = new Transform3d(Inches.of(2.644), Inches.of(11.784437),
            Inches.of(26.531608), new Rotation3d(Degrees.of(0), Degrees.of(3.7 + 16.5), Degrees.of(20)));
        public static final Transform3d BackCamera = new Transform3d(Inches.of(-7.578), Inches.of(10.052),
            Inches.of(27.982014), new Rotation3d(Degrees.of(0), Degrees.of(3.7 - 32), Degrees.of(-190)));
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