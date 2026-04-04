package frc.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Translation2d;

import org.littletonrobotics.junction.Logger;
import java.util.List;

public class BallSim {

  public Pose3d landedPose3d; 

  private static BallSim instance;

      public static BallSim getInstance() {
      if (instance == null)
        instance = new BallSim();
      return instance;
    }

    private static final double kG = 9.80665; // m/s^2

    private boolean active = false;
    private Translation3d pos = new Translation3d();
    private Translation3d vel = new Translation3d();
    private double lastT = 0.0;
    private List<Pose3d> arc = new java.util.ArrayList<>();

    /** Start a throw from an initial pose, with roll/pitch/yaw and a launch speed (m/s). */
    public void throwBall(Pose3d startPose, Rotation3d rpy, double speedMps, Translation2d robotVelocity) {
      // Launch direction is robot +X rotated by the given RPY (roll doesn’t matter for a sphere, but harmless).
      Rotation3d corrected = new Rotation3d(0, -rpy.getY(), rpy.getZ());
      Translation3d dir = new Translation3d(1, 0, 0).rotateBy(corrected);
      
      //reset arc to have nothing
      arc = new java.util.ArrayList<>();

      pos = startPose.getTranslation();
      vel = new Translation3d(
        dir.getX() * speedMps + robotVelocity.getX(), 
        dir.getY() * speedMps + robotVelocity.getY(), 
        dir.getZ() * speedMps);

      active = true;
      lastT = Timer.getFPGATimestamp();
    }

    /** Advance physics one step. Call from simulationPeriodic(). */
    public void update() {
      if (!active) {
        landedPose3d = getPose(); 
        return; 
      };

      double t = Timer.getFPGATimestamp();
      double dt = t - lastT;
      lastT = t;

      // Integrate velocity (gravity) then position
      vel = new Translation3d(vel.getX(), vel.getY(), vel.getZ() - kG * dt);
      pos = new Translation3d(
          pos.getX() + vel.getX() * dt,
          pos.getY() + vel.getY() * dt,
          pos.getZ() + vel.getZ() * dt
      );

      arc.add(new Pose3d(pos, new Rotation3d()));
      Pose3d[] send_arc = arc.toArray(new Pose3d[0]);

      Logger.recordOutput("Shooter/ball_vector", new Pose3d[] {new Pose3d(pos, new Rotation3d())});

      // Stop when it hits the floor (z <= 0). Adjust if your “ground” is different.
      if (pos.getZ() <= 0.0) {
        pos = new Translation3d(pos.getX(), pos.getY(), 0.0);
        active = false;
      }
    }

    public boolean isActive() {
      return active;
    }

    public Pose3d getPose() {
      // Orientation doesn’t really matter for a ball visualization
      return new Pose3d(pos, new Rotation3d());
    }

    public Pose3d getLandedPose3d() {
      return landedPose3d; 
    }
}
