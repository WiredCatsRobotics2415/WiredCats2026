package frc.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.shooter.Shooter;
import frc.visualization.BallSim;
import frc.constants.Measurements; 

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Turret extends SubsystemBase {
    // @Getter private TurretIO io; 
    private static Turret instance; 
    private static Shooter shooter = Shooter.getInstance();
    private static BallSim ball = BallSim.getInstance();
    
    public static Turret getInstance() {
        if (instance == null) instance = new Turret(); 
        return instance; 
    }

    // right now this just gets the angle to turn the turret to point it at the hub
    // in order to account for robot movement we would have to predict the robot's next position
    // but I didn't want to code that until we discussed it at the next meeting

    public double calculateTurretAngleToHub(Pose2d robotPose) {
        // Start with static turret angle (robot-relative angle to hub)
        double turretAngle = calculateStaticTurretAngle(robotPose);

        Shooter shooter = Shooter.getInstance();
        // Get velocity-compensated shooter speed
        double needed = shooter.getSpeedToHubForPoseAndVelocities(robotPose);

        CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
        ChassisSpeeds currVelocities = drive.getVelocity();

        ChassisSpeeds fieldRelativeVelocities = ChassisSpeeds.fromRobotRelativeSpeeds(
            currVelocities,
            robotPose.getRotation()
        );

        // Calculate the absolute angle to hub in field frame
        Pose2d hubPose = Measurements.HubLocation;
        double angleToHub = hubPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians();

        // Calculate perpendicular velocity component (perpendicular to shot direction in field frame)
        double perpendicularVector = -fieldRelativeVelocities.vxMetersPerSecond * Math.sin(angleToHub)
                                    + fieldRelativeVelocities.vyMetersPerSecond * Math.cos(angleToHub);

        Logger.recordOutput("Turret/Perpendicular Vel", perpendicularVector);
        Logger.recordOutput("Turret/Needed Speed", needed);

        // Calculate angle adjustment with safety check for Math.asin domain
        double angleAdjustment = 0.0;
        if (needed > 0.001) { // Avoid division by zero
            // Clamp the ratio to [-1, 1] to prevent NaN from Math.asin
            double sinValue = perpendicularVector / needed;
            sinValue = Math.max(-1.0, Math.min(1.0, sinValue));

            angleAdjustment = Math.asin(sinValue);

            // Log if we hit the clamp (indicates robot moving too fast for shooter speed)
            if (Math.abs(perpendicularVector / needed) > 1.0) {
                Logger.recordOutput("Turret/Warning", "Perpendicular velocity exceeds shooter speed!");
            }
        }

        double adjustedAngle = turretAngle + angleAdjustment;

        double distanceFromHub = robotPose.getTranslation().getDistance(Measurements.HubLocation.getTranslation());

        double angle = 90-15;

        if (distanceFromHub <= Measurements.ShooterHubRegionOne) {
            angle = 90-15;
        } else {
            angle = 90-45;
        }

        // Throw ball for visualization with the adjusted angle
        ball.throwBall(
            new Pose3d(robotPose.getX(), robotPose.getY(), 0.457, new Rotation3d(0, 0, 0)),
            new Rotation3d(0, Math.PI - Math.toRadians(180+angle), robotPose.getRotation().getRadians() + adjustedAngle),
            needed
        );

        Logger.recordOutput("ACTUAL ANGLE IN RADIANS (# of pis)", Math.toRadians(180+angle));

        Logger.recordOutput("Turret/Robot Pose", robotPose);
        Logger.recordOutput("Turret/Static Turret Ang (rad)", turretAngle);
        Logger.recordOutput("Turret/Ang Adjustment (rad)", angleAdjustment);
        Logger.recordOutput("Turret/Adj Turret Ang (rad)", adjustedAngle);

        // Convert to degrees for logging
        double adjustedAngleDeg = Math.toDegrees(adjustedAngle);
        double angleAdjustmentDeg = Math.toDegrees(angleAdjustment);

        Logger.recordOutput("Turret/Adj Turret Ang", adjustedAngleDeg);
        Logger.recordOutput("Turret/Robot Rotation (deg)", robotPose.getRotation().getDegrees());
        Logger.recordOutput("Turret/Ang Offset", angleAdjustmentDeg);

        return adjustedAngleDeg;
    }

    public double calculateStaticTurretAngle(Pose2d robotPose) {
        Pose2d hubPose = Measurements.HubLocation;

        double angleToHub = hubPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians();
        double turretAngle = angleToHub - robotPose.getRotation().getRadians() + Math.toRadians(Measurements.TurretAngleOffset); // 0 right now

        Logger.recordOutput("Turret/Robot Ang to Hub", angleToHub); 
        Logger.recordOutput("Turret/Static Turret Ang", turretAngle); 

        return turretAngle; 
    }
}
