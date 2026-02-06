package frc.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.subsystems.shooter.Shooter;
import frc.visualization.BallSim;
import frc.constants.Measurements;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Turret extends SubsystemBase {
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
        // Get velocity-compensated shooter speed and angle
        double[] speedAndAngle = shooter.getShooterSpeedAndAngleToHub(robotPose);
        double speedMagnitude = speedAndAngle[0];
        double angleInFieldFrame = speedAndAngle[1];

        // Calculate turret angle in robot frame
        double turretAngleInRadians = angleInFieldFrame - robotPose.getRotation().getRadians() + Math.toRadians(Measurements.TurretAngleOffset);

        double angle = 90-shooter.getDegreesAngleForPose(robotPose);

        shooter.setGoalSpeed(speedMagnitude);

        if (!Robot.isReal()) {
                // Throw ball for visualization with the adjusted angle
                ball.throwBall(
            new Pose3d(robotPose.getX(), robotPose.getY(), 0.457, new Rotation3d(0, 0, 0)),
            new Rotation3d(0, Math.PI - Math.toRadians(180+angle), angleInFieldFrame),
            speedMagnitude
        );
        }

        Logger.recordOutput("ACTUAL ANGLE IN RADIANS (# of pis)", Math.toRadians(180+angle));

        Logger.recordOutput("Turret/Robot Pose", robotPose);
        Logger.recordOutput("Turret/Angle In Field Frame (rad)", angleInFieldFrame);
        Logger.recordOutput("Turret/Turret Angle (rad)", turretAngleInRadians);
        Logger.recordOutput("Turret/Turret Angle (deg)", Math.toDegrees(turretAngleInRadians));
        Logger.recordOutput("Turret/Robot Rotation (deg)", robotPose.getRotation().getDegrees());
        Logger.recordOutput("Turret/Shooter Speed", speedMagnitude);

        return Math.toDegrees(turretAngleInRadians);
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
