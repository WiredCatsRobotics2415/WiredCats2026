package frc.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.shooter.Shooter;
import frc.constants.Measurements; 

import lombok.Getter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Turret extends SubsystemBase {
    // @Getter private TurretIO io; 
    private static Turret instance; 
    
    public static Turret getInstance() {
        if (instance == null) instance = new Turret(); 
        return instance; 
    }

    // right now this just gets the angle to turn the turret to point it at the hub
    // in order to account for robot movement we would have to predict the robot's next position
    // but I didn't want to code that until we discussed it at the next meeting

    // TODO: update to account for robot motion
    public double calculateStaticTurretAngle(Pose2d robotPose) {
        Pose2d hubPose = Measurements.HubLocation; 

        double angleToHub = hubPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getDegrees(); 
        double turretAngle = angleToHub - robotPose.getRotation().getDegrees() + Measurements.TurretAngleOffset; 

        return turretAngle; 
    }
}
