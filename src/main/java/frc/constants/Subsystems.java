package frc.constants;

// this is all placeholder from last year
import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalSource;
import frc.utils.tuning.TuneableNumber;

public class Subsystems {
    public static class DriveConstants {

        public static final double HeadingKA = 0;

    }

    public static class VisionConstants {
        public static final String[] PoseEstimationLLNames = {
            "limelight-front-left", // or whatever your names are
            "limelight-front-right",
            "limelight-back"
        };
        
        public static final String FrontLeftName = "limelight-front-left";
        public static final String FrontRightName = "limelight-front-right";
        public static final String BackCenterName = "limelight-back";
        public static final String TurretName = "limelight-turret"; // or whatever
        
        public static final List<Integer> HUB_APRILTAG_IDS = List.of(/* tag IDs */);
        
        public static class LimelightsForElements {
            public int[] indexInPEList;
            // constructor, etc.
        }
        
        public static final double ThreeGDiagonalFOV = 63.3; // degrees, for sim
    }

    public static class ShooterConstants {
        public static final int FLYWHEEL_1_ID = 1; //TODO: Placeholder ID values, change when systems connects them
        public static final int FLYWHEEL_2_ID = 2;  //Placeholder
        public static final int INTAKE_MOTOR_ID = 1; //Placeholder
        public static final float INTAKE_SPEED = 5; //Placeholder, Intake Speed is currently in RPS may want to change later
        public static double kSVolts = 0.0;
        public static double kVVoltSecondsPerRotation = 0.0;
        public static double kP = 0.5; // PID proportional gain for feedback control
        public static double kShooterToleranceRPS = 0.0;
        public static double kEncoderDistancePerPulse = 0.0;
        // Feedforward gains - these provide a model-based estimate of voltage needed
        public static double kS = 0.1; // Voltage to overcome static friction (volts)
        public static double kV = 0.12; // Voltage per unit velocity (volts per RPS) - tune this!
        public static double kA = 0.01; // Voltage per unit acceleration (volts per RPS/s)
    }

    public static class ClimberConstants {
        public static final double GoalDeadband = 0;
        public static double kSVolts;
        public static double kVVoltSecondsPerRotation;
        public static double kP;
    }
}
