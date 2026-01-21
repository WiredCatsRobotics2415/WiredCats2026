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
import frc.utils.tuning.TuneableNumber;

public class Subsystems {
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
}