// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems.shooter;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;
import frc.subsystems.shooter.ShooterSim;

import org.apache.commons.math3.analysis.function.Sqrt;
import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Measurements;
import frc.constants.Subsystems.ShooterConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.turret.Turret;
import lombok.Setter;
import frc.visualization.BallSim;

// Import Subsystems Constants, TODO: currently all placeholders change once we have real values
public class Shooter extends SubsystemBase {

  private static Shooter instance = null;
  private static TalonFX flywheel1;
  private static TalonFX flywheel2;
  private static TalonFX indexerMotor;
  private static Integer FLYWHEEL_1_ID = 1;
  private static Integer FLYWHEEL_2_ID = 2;
  private static Integer INTAKE_MOTOR_ID = 3;

  private static BallSim ball = BallSim.getInstance();
  private boolean ballWasInAir = false; // used for getting distance ball lands from target

  //WE ARE USING SIMPLE FEEDFORWARD AND PID BECAUSE WE ARE SETTING FLYWHEEL VELOCITIES AND DON'T NEED POSITION ACCURACY
  private final PIDController pid = new PIDController(ShooterConstants.kP, 0.0, 0.0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

  private static ShooterSim sim = new ShooterSim();

  // for both sim and real, to store the pitch angle (in radians), turret angle, and needed speed (all from most recent calculation)
  // also store angle in field frame for sim specifically
  private double lastCalculationPitchRadians = 0.0; 
  private double lastCalculatedNeededSpeed = 0.0; 
  private double lastCalculatedAngleInFieldFrame = 0.0; // sim only
  private double lastCalculationTurretAngleDegrees = 0.0;

  public double getLastAngle() {
    return lastCalculationTurretAngleDegrees;
  }

  private double goalSpeed = 0.0;

  // Intake Functions Below
  private Shooter() {
    // Constructor, idk what to put here rn
    flywheel1 = new TalonFX(FLYWHEEL_1_ID);
    flywheel2 = new TalonFX(FLYWHEEL_2_ID);
    indexerMotor = new TalonFX(INTAKE_MOTOR_ID);
  }

  public static Shooter getInstance() {
    if (instance == null)
      instance = new Shooter();
    return instance;
  }

  public void startShooting() // Starts Intake
  {
    if (Robot.isReal()) {
      //starting it at 50%
      indexerMotor.set(0.5);
      // shoot in real? can use lastCalculationPitchRadians, lastCalculationTurretAngleRadians, lastCalculatedNeededSpeed
    } else {
      ballWasInAir = false; // reset to false in case it landed before
      CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
      Pose2d robotPose = drive.getPose(); 
      setTurretAndShooterForPose(robotPose); // ensures up-to-date

      ball.throwBall(
          new Pose3d(robotPose.getX(), robotPose.getY(), Measurements.ShooterHeightFromGround, new Rotation3d()),
          new Rotation3d(0, lastCalculationPitchRadians, lastCalculatedAngleInFieldFrame),
          lastCalculatedNeededSpeed
      );
    }
  }

  public void stopShooting() // Stops Intake
  {
    indexerMotor.set(0.0);
  }

  // Flywheel Functions Below
  public float getSpeed() // Returns average speed of both flywheels
  {
    if (Robot.isReal()) {
    return (float) ((flywheel1.getVelocity().getValueAsDouble() + flywheel2.getVelocity().getValueAsDouble()) / 2);
    } else {
      // In simulation, return the actual velocity converted to voltage-like units for PID
      // This matches the units expected by the PID controller
      return (float) ((sim.getFlywheel1VelocityRPS() + sim.getFlywheel2VelocityRPS()) / 2);
    }
  }

  public double getGoalSpeed() {
    return goalSpeed;
  }

  public void setGoalSpeed(double goalSpeed) {
    this.goalSpeed = goalSpeed;
  }

    public int getAngleForPose(Pose2d robotPose) {
      int angle; 

      if (Measurements.ShootIntoHubRegion.contains(robotPose.getTranslation())) {
          angle = Measurements.ShooterAngleLow; // closer to hub (inside region)
      } else {
          angle = Measurements.ShooterAngleHigh; // further from hub (outside region)
      }

      return angle; 
    }

    public Pose2d getTarget(Pose2d robotPose) {
      int shooterAngle = getAngleForPose(robotPose); 
      if (shooterAngle != 45) {
            return Measurements.HubLocation;
        }
        // low angle, selects closest target for left or right
        return (robotPose.getTranslation().getDistance(Measurements.LeftMidAllianceRegion.getTranslation()) < 
                robotPose.getTranslation().getDistance(Measurements.RightMidAllianceRegion.getTranslation())) 
                ? Measurements.LeftMidAllianceRegion : Measurements.RightMidAllianceRegion;
    }

    private double calculateShooterSpeedRequired(Pose2d robotPose, Translation3d shooterSpeedVector, double horizontalDistance) {
      double pitch = Math.toRadians((double) getAngleForPose(robotPose));
      double[] ts = calculateInitialSpeedAndImpactTime(pitch, shooterSpeedVector, horizontalDistance); 
      double t = ts[0]; 
      double speed = ts[1]; 

      return speed; 
    }

    private double calculateTurretAngle(Pose2d robotPose, Translation3d shooterSpeedVector) {
      double fieldRelative = getAngleInFieldFrame(shooterSpeedVector);  
      double robotHeadingAngle = robotPose.getRotation().getRadians(); 
      return fieldRelative - robotHeadingAngle; 
    }

    private Translation3d getRobotVector(Pose2d robotPose) {
      CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
      ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getVelocity(), robotPose.getRotation());
      Translation3d robotVelocity3d = new Translation3d(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond, 0.0);
      Logger.recordOutput("Shooter/robotVelocityVector", makeAdvantageScopeLine(robotVelocity3d, robotPose)); 
      return robotVelocity3d; 
    }

    private double getTargetHeight(Pose2d target) {
      if (target == Measurements.LeftMidAllianceRegion || target == Measurements.RightMidAllianceRegion) {
        return 0.0; 
      }
      return Measurements.HubGoalHeight; 
    }

    private Translation3d getVectorFromRobotToTarget(Pose2d robotPose, Pose2d target) {
      Translation3d ballPositionVector = new Translation3d(robotPose.getX(), robotPose.getY(), Measurements.ShooterHeightFromGround);
      Translation3d targetGoalPositionVector = new Translation3d(target.getX(), target.getY(), getTargetHeight(target)); 
      Logger.recordOutput("Shooter/targetPose", targetGoalPositionVector); // this is correct/what I want

      Translation3d vectorFromRobotToTarget = targetGoalPositionVector.minus(ballPositionVector); 
      return vectorFromRobotToTarget; 
    }

    private Translation3d getShooterSpeedVector(Pose2d robotPose, Pose2d target) {
      Translation3d robotVelocity3d = getRobotVector(robotPose); // m/s
      Translation3d vectorFromRobotToTarget = getVectorFromRobotToTarget(robotPose, target); // meters

      double dx = vectorFromRobotToTarget.getX();
      double dy = vectorFromRobotToTarget.getY();
      double horizontalDist = Math.sqrt(dx*dx + dy*dy);

      Translation3d direction = new Translation3d(
          dx / horizontalDist,
          dy / horizontalDist,
          vectorFromRobotToTarget.getZ()
      );

      return direction.minus(robotVelocity3d);
    }

    public Pose3d[] mapTrajectory(double initialSpeed, double pitchAngle, Rotation3d direction, Pose2d robotPose) {
      Pose3d[] trajectory = new Pose3d[99];
      double robotX = robotPose.getX();
      double robotY = robotPose.getY();
      double yaw = direction.getZ(); // field-frame angle

      double dt = 0.05; // seconds per step

      for (int i = 0; i < 99; i++) {
          double t = i * dt;

          // horizontal distance traveled at this time step
          double horizontalDist = initialSpeed * Math.cos(pitchAngle) * t;

          // vertical: s*sin(pitch)*t - 4.8t^2 (matching your existing equations)
          double z = Measurements.ShooterHeightFromGround 
                  + (initialSpeed * Math.sin(pitchAngle) * t) 
                  - (4.8 * t * t);

          double x = robotX + horizontalDist * Math.cos(yaw);
          double y = robotY + horizontalDist * Math.sin(yaw);

          trajectory[i] = new Pose3d(x, y, z, direction);
      }

      return trajectory;
  }

    private double[] calculateInitialSpeedAndImpactTime(double pitch, Translation3d shooterCompensationVector, double horizontalDist) {
      // horizontal = t * s * cos(pitch)
      // vertical = t * s * sin(pitch) - 4.8t^2
      
      double vertical = shooterCompensationVector.getZ();
      
      double ts = horizontalDist / Math.cos(pitch); 
      double radicand = ((ts * Math.sin(pitch)) - vertical) / 4.8;

      if (radicand < 0) {
          Logger.recordOutput("Shooter/ERR_neg_radicand", radicand);
          return new double[]{Double.NaN, Double.NaN};
      }

      double t = Math.sqrt(radicand);

      // double t = Math.sqrt(((ts * Math.sin(pitch)) - vertical) / 4.8); 
      double s = ts / t; 
      double[] result = new double[]{t, s}; 
      Logger.recordOutput("Shooter/t & s", result); 

      return result; 
    }

    // TODO: figure out why the hub having a goal height is causing the radicant to be negative
    private double getDistBallFromTarget(Pose2d target) {
      Pose3d Target3D = new Pose3d(target.getX(), target.getY(), getTargetHeight(target), new Rotation3d(0, 0,0)); 
      Pose3d landedBallPose = ball.getLandedPose3d(); 
      if (landedBallPose == null) {
        return Double.NaN; 
      }
      return landedBallPose.getTranslation().getDistance(Target3D.getTranslation()); 
    }

    public Pose3d[] makeAdvantageScopeLine(Translation3d vector, Pose2d robotPose) {
      Pose3d[] final_array = new Pose3d[9]; 
      double robotX = robotPose.getX(); 
      double robotY = robotPose.getY(); 
      for (int i = 0; i < 9; i++) {
        Pose3d to_add = new Pose3d(vector.getX()/(i+1) + robotX, vector.getY()/(i+1) + robotY, vector.getZ()/(i+1), new Rotation3d(0,0,0)); 
        final_array[i] = to_add; 
      }
      return final_array; 
    }

    private double getAngleInFieldFrame(Translation3d shooterSpeedVector) {
      return Math.atan2(shooterSpeedVector.getY(), shooterSpeedVector.getX());
    }

    private void setTurretAndShooterForPose(Pose2d robotPose) {
      double pitchAngle = getAngleForPose(robotPose); 
      Pose2d target = getTarget(robotPose); 

      Logger.recordOutput("Shooter/pitchAngle", pitchAngle); 
      Logger.recordOutput("Shooter/target", target); 

      Translation3d shooterSpeedVector = getShooterSpeedVector(robotPose, target); 
      Logger.recordOutput("Shooter/shooterSpeedVector", makeAdvantageScopeLine(shooterSpeedVector, robotPose)); 

      Translation3d robotToTarget = getVectorFromRobotToTarget(robotPose, target); 
      Logger.recordOutput("Shooter/robotToTarget", makeAdvantageScopeLine(robotToTarget, robotPose)); 
      double dx = robotToTarget.getX();
      double dy = robotToTarget.getY();

      double horizontalDist = Math.sqrt(dx*dx + dy*dy);
      Logger.recordOutput("Shooter/horizontalDist", horizontalDist); 

      double speed = calculateShooterSpeedRequired(robotPose, shooterSpeedVector, horizontalDist); 
      Logger.recordOutput("Shooter/speed", speed); 

      double turretAngle = calculateTurretAngle(robotPose, shooterSpeedVector); 
      Logger.recordOutput("Shooter/turretAngle", turretAngle); 

      Logger.recordOutput("Shooter/origin", new Translation3d(0.0, 0.0, 0.0)); 
      
      lastCalculationPitchRadians = Math.toRadians((double) pitchAngle) - Math.PI/2; // related to how the ball shooting is configured
      lastCalculatedNeededSpeed = speed; 
      lastCalculatedAngleInFieldFrame = getAngleInFieldFrame(shooterSpeedVector); 
      lastCalculationTurretAngleDegrees = turretAngle; 

      setGoalSpeed(speed); 

      //Logger.recordOutput("Shooter/ballTrajectory", 
        //mapTrajectory(speed, Math.toRadians(pitchAngle), 
        //new Rotation3d(0, 0, lastCalculatedAngleInFieldFrame), robotPose));

      Logger.recordOutput("Shooter/ballToTarget", getDistBallFromTarget(target)); // updates when landed
    }

    @Override
    public void periodic() {
      Logger.recordOutput("Shooter/goalSpeed", goalSpeed);
      Logger.recordOutput("Shooter/actualSpeed", getSpeed());

      // Combine feedforward (estimates needed voltage) with feedback (corrects for errors)
      double feedforwardVolts = feedforward.calculate(goalSpeed);
      double feedbackVolts = pid.calculate(getSpeed(), goalSpeed);
      double voltage = feedforwardVolts + feedbackVolts;

      Logger.recordOutput("Shooter/feedforwardVolts", feedforwardVolts);
      Logger.recordOutput("Shooter/feedbackVolts", feedbackVolts);
      Logger.recordOutput("Shooter/totalVoltage", voltage);

      Logger.recordOutput("Shooter/isInShootingMode", RobotContainer.getInstance().isInShootingMode()); 
      
      if (Robot.isReal()) {
          flywheel1.setVoltage(voltage);
          flywheel2.setVoltage(voltage);
      } else {
          sim.setFlywheel1Voltage(voltage);
          sim.setFlywheel2Voltage(voltage);
          sim.update(0.02);
          ball.update();
      }

      CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
      Pose2d robotPose = drive.getPose(); 

      if (RobotContainer.getInstance().isInShootingMode()) {
          setTurretAndShooterForPose(robotPose); // in periodic so updates
      }
      
      boolean isInAir = ball.isActive();
      if (ballWasInAir && !isInAir) { // if the ball was previously in the air and was no longer
          Pose2d target = getTarget(robotPose);
          Logger.recordOutput("Shooter/ballToTarget", getDistBallFromTarget(target));
          ballWasInAir = isInAir; // update was in air to false so it doesn't run again
      }
  }
}
