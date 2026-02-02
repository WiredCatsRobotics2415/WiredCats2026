package frc.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.shooter.Shooter;
import frc.constants.Measurements; 

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

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

    // umm i think this is right? 
    public double calculateTurretAngleToHub(Pose2d robotPose) {
        double turretAngle = calculateStaticTurretAngle(robotPose); 

        Shooter shooter = Shooter.getInstance(); 
        double needed = shooter.getSpeedToHubForPoseAndVelocities(robotPose);  

        CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance(); 
        ChassisSpeeds currVelocities = drive.getVelocity(); 

        // perpendicular velocity component
        double perpendicularVector = -currVelocities.vxMetersPerSecond * Math.sin(turretAngle) + currVelocities.vyMetersPerSecond * Math.cos(turretAngle); 

        double angleAdjustment = Math.toDegrees(Math.asin(perpendicularVector/needed)); 

        double adjustedAngle = Math.toDegrees(turretAngle) + angleAdjustment; 

        Logger.recordOutput("Turret/Robot Pose", robotPose); 
        
        Logger.recordOutput("Turret/Adj Turret Ang", adjustedAngle); 
        Logger.recordOutput("Turret/Robot Rotation (deg)", robotPose.getRotation().getDegrees());
        Logger.recordOutput("Turret/Ang Offset", angleAdjustment); 

        return adjustedAngle; 
    }

    public double calculateStaticTurretAngle(Pose2d robotPose) {
        Pose2d hubPose = Measurements.HubLocation;

        double angleToHub = hubPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians();
        double turretAngle = angleToHub - robotPose.getRotation().getRadians() + Math.toRadians(Measurements.TurretAngleOffset); 

        Logger.recordOutput("Turret/Robot Ang to Hub", angleToHub); 
        Logger.recordOutput("Turret/Static Turret Ang", turretAngle); 

        return turretAngle; 
    }
}
