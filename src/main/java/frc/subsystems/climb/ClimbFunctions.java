package frc.subsystems.climb;

import frc.subsystems.drive.CommandSwerveDrivetrain;

public class ClimbFunctions {
    private static Climber climber;
    private static CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();

    public void climb(boolean level) {
        
    }

    public void climbAlreadyAligned(boolean isLeftFromDriverStation, boolean allianceColorIsRed) {
        double forwardSpeed = 0.0;
        double sideSpeed = 0.0;
        
        if (isLeftFromDriverStation && allianceColorIsRed) {
            //red left
            forwardSpeed = 0.5;
            sideSpeed = 0.5;

        } else if (isLeftFromDriverStation && !allianceColorIsRed) {
            //blue left
            forwardSpeed = -0.5;
            sideSpeed = -0.5;
            
        } else if (!isLeftFromDriverStation && allianceColorIsRed) {
            //red right
            forwardSpeed = -0.5;
            sideSpeed = 0.5;
            
        } else {
            //blue right
            forwardSpeed = 0.5;
            sideSpeed = -0.5;

        }

        while (!climber.getForwardLimit()) {
                drive.driveOpenLoopFieldCentricRequest.withVelocityX(sideSpeed).withVelocityY(0).withRotationalRate(0.0);
        } 
        drive.driveOpenLoopFieldCentricRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0.0);

        while (!climber.getSideLimit()) {
                drive.driveOpenLoopFieldCentricRequest.withVelocityX(0).withVelocityY(forwardSpeed).withRotationalRate(0.0);
        }

        drive.driveOpenLoopFieldCentricRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0.0);
        
    }

    public void alignAndClimb() {

    }
}
