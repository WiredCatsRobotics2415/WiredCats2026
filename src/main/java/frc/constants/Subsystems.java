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
        public static final double PPTranslationP = 20;
         public static final double RotationP = 15;
        public static final double HeadingKA = 0;
        public static PIDConstants PPTranslationPID = new PIDConstants(PPTranslationP, 0, 0); // test 3: kp 1, test 4-: kp 5, test 13-: kp 10, test 15-: kp 7, test 19-: kp 5, test 21: kp 7
        public static PIDConstants RotationPID = new PIDConstants(RotationP, 0, 0);
        public static PathFollowingController PathFollowingController = new PPHolonomicDriveController(PPTranslationPID,
            RotationPID);
    }

    public static class IntakeConstants {

        public static final double HeadingKA = 0;
        public static double kSwitchThreshold;

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

    public static class PortNumbers {
        public static final int NumpadPort = 1; 
        public static final int ControllerPort = 4; 
        public static final int Flywheel_1_ID = 1; 
        public static final int Flywheel_2_ID = 2; 
        public static final int Intake_Motor_ID = 3; 
        public static final int Handoff_Motor_ID = 4; 
        public static final int Turret_Motor = 5; 
        public static final int Climb_Motor = 6; 
        public static final int Intake_Front_Limit_ID = 0; 
        public static final int Intake_Back_Limit_ID = 1; 
    }

    public static class ShooterConstants {
        public static final int FLYWHEEL_1_ID = 53; //TODO: Placeholder ID values, change when systems connects them
        public static final int FLYWHEEL_2_ID = 54;  //Placeholder
        public static final int INTAKE_MOTOR_ID = 55; //Placeholder
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
        public static final double MaxHeight = 0.5;
        public static double kSVolts;
        public static double kVVoltSecondsPerRotation;
        public static double kP = 1;
        public static double kG = 0.3;
        public static double kClimbArmMassKg = 1;
        public static double kClimbArmLengthM = 1;
        public static double kClimbArmMOIkgm2 = 1;
    }

    public static class TurretConstants {
        public static final double GoalDeadband = 0;
        public static double kSVolts;
        public static double kVVoltSecondsPerRotation;
        public static double kP = 0.01;
        public static double kMaxVelocity = 1;
        public static double kMaxAcceleration = 1;
        public static double kI = 0;
        public static double kD = 8;
        //do not set this to 0!!!
        public static double kDt = 0.1;
    }
}
