package frc.visualization;

import static edu.wpi.first.units.Units.Radian;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.subsystems.climb.Climber;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.shooter.Shooter;
import frc.subsystems.turret.Turret;

public class RobotParts {
    private final Shooter shooter = Shooter.getInstance();
    private final Turret turret = Turret.getInstance();
    private final Climber climber = Climber.getInstance();
    private final CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private static RobotParts instance;

      public static RobotParts getInstance() {
            if (instance == null)
      instance = new RobotParts();
            return instance;
        }

    public Pose3d turretPose3D() {
        double turretSimAngle = turret.sim.getAngleRadians();
        return new Pose3d(-0.1, -0.3, 0.35, new Rotation3d(Math.toRadians(90), 0, turretSimAngle));
    }

    public Pose3d intakePose3D() {
        return new Pose3d(0.25, 0.25, 0.1, new Rotation3d(0, 0, Math.toRadians(90)));
    }

    public Pose3d climberPose3D() {
        double height = climber.getHeight();
        return new Pose3d(0, 0.25, height, new Rotation3d(0, 0, Math.toRadians(90)));
    }

    public void update() {
        Logger.recordOutput("Turret/Pose", turretPose3D());
        Logger.recordOutput("Intake/Pose", intakePose3D());
        Logger.recordOutput("Climber/Pose", climberPose3D());
    }



    }

