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
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Measurements;
import frc.constants.Subsystems.ShooterConstants;
import frc.constants.Subsystems.TurretConstants;
import frc.constants.Subsystems.PortNumbers;
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
  private static TalonFX handoffMotor;
  private static Integer FLYWHEEL_1_ID = PortNumbers.Flywheel_1_ID;
  private static Integer FLYWHEEL_2_ID = PortNumbers.Flywheel_2_ID;
  private static Integer INTAKE_MOTOR_ID = PortNumbers.Intake_Motor_ID;
  private static Integer HANDOFF_MOTOR_ID = PortNumbers.Handoff_Motor_ID;

  private static BallSim ball = BallSim.getInstance();
  private boolean ballWasInAir = false; // used for getting distance ball lands from target

  //WE ARE USING SIMPLE FEEDFORWARD AND PID BECAUSE WE ARE SETTING FLYWHEEL VELOCITIES AND DON'T NEED POSITION ACCURACY
 private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(ShooterConstants.kMaxVelocity.get(), ShooterConstants.kMaxAcceleration.get());
    private final ProfiledPIDController pid =
      new ProfiledPIDController(ShooterConstants.kP.get(),0, ShooterConstants.kD.get(), constraints, 0.02);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS.get(), ShooterConstants.kV.get(), ShooterConstants.kA.get());

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
    //fix this later
    flywheel1 = new TalonFX(FLYWHEEL_1_ID);
    flywheel2 = new TalonFX(FLYWHEEL_2_ID);
    indexerMotor = new TalonFX(19);
    handoffMotor = new TalonFX(16);
  }

  public static Shooter getInstance() {
    if (instance == null)
      instance = new Shooter();
    return instance;
  }

  public void startShooting() // Starts Intake
  {
    System.out.println("START SHOOTING!!");
    if (Robot.isReal()) {
      //starting it at 50%
      handoffMotor.set(-1);
      indexerMotor.set(0.5);
      // shoot in real? can use lastCalculationPitchRadians, lastCalculationTurretAngleRadians, lastCalculatedNeededSpeed
    } else {
      ballWasInAir = false; // reset to false in case it landed before
      CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
      Pose2d robotPose = drive.getPose(); 
      setTurretAndShooterForPose(robotPose); // ensures up-to-date

      ChassisSpeeds fieldVel = ChassisSpeeds.fromFieldRelativeSpeeds(drive.getVelocity(), robotPose.getRotation());
      Translation2d robotVelocity = new Translation2d(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond);

      ball.throwBall(
          new Pose3d(robotPose.getX(), robotPose.getY(), Measurements.ShooterHeightFromGround, new Rotation3d()),
          new Rotation3d(0, lastCalculationPitchRadians, lastCalculatedAngleInFieldFrame),
          lastCalculatedNeededSpeed, robotVelocity
      );
    }
  }

  public void stopShooting() // Stops Intake
  {
    System.out.println("STOP SHOOTING!!!");
    handoffMotor.set(0);
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
    if (goalSpeed>0) {
      this.goalSpeed = goalSpeed;
    }
  }

  // auto-targeting
    public double getAngleForPose(Pose2d robotPose) {
      boolean inHub = Measurements.ShootIntoHubRegion.contains(robotPose.getTranslation()); 
      Logger.recordOutput("Shooter/inHubRegion", inHub);
      Logger.recordOutput("Shooter/robotPoseForAngle", robotPose);
      if (inHub) {
          return Math.toRadians(90 - Measurements.ShooterAngleHigh);
      } else {
          return Math.toRadians(Measurements.ShooterAngleLow);
      }
  }

    public Pose2d getTarget(Pose2d robotPose) {
      if (Measurements.ShootIntoHubRegion.contains(robotPose.getTranslation())) {
        return Measurements.HubLocation;
      }
        // if not in shoot into hub region
        return (robotPose.getTranslation().getDistance(Measurements.LeftMidAllianceRegion.getTranslation()) < 
                robotPose.getTranslation().getDistance(Measurements.RightMidAllianceRegion.getTranslation())) 
                ? Measurements.LeftMidAllianceRegion : Measurements.RightMidAllianceRegion;
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
      Logger.recordOutput("Shooter/robotVelocityVector", makeAdvantageScopeLine(robotVelocity3d, robotPose, 0)); 
      return robotVelocity3d; 
    }

    private double getTargetHeight(Pose2d target) {
      if (target == Measurements.HubLocation) {
        return Measurements.HubGoalHeight;
      }
      return 0.0;
    }

    // Source - https://stackoverflow.com/a/37960741
    // Posted by Salix alba
    // Retrieved 2026-04-03, License - CC BY-SA 3.0

  public static double[] solveRealQuarticRoots(double a, double b, double c, double d, double e) {
    double s1 = 2 * c * c * c - 9 * b * c * d + 27 * (a * d * d + b * b * e) - 72 * a * c * e;
    double q1 = c * c - 3 * b * d + 12 * a * e;
    double discrim1 = -4 * q1 * q1 * q1 + s1 * s1;
    if (discrim1 > 0) {
      double s2 = s1 + Math.sqrt(discrim1);
      double q2 = Math.cbrt(s2 / 2);
      double s3 = q1 / (3 * a * q2) + q2 / (3 * a);
      double discrim2 = (b * b) / (4 * a * a) - (2 * c) / (3 * a) + s3;
      if (discrim2 > 0) {
        double s4 = Math.sqrt(discrim2);
        double s5 = (b * b) / (2 * a * a) - (4 * c) / (3 * a) - s3;
        double s6 = (-(b * b * b) / (a * a * a) + (4 * b * c) / (a * a) - (8 * d) / a) / (4 * s4);
        double discrim3 = (s5 - s6);
        double discrim4 = (s5 + s6);
        // actual root values, may not be set
        double r1 = Double.NaN, r2 = Double.NaN, r3 = Double.NaN, r4 = Double.NaN;

        if (discrim3 > 0) {
          double sqrt1 = Math.sqrt(s5 - s6);
          r1 = -b / (4 * a) - s4 / 2 + sqrt1 / 2;
          r2 = -b / (4 * a) - s4 / 2 - sqrt1 / 2;
        } else if (discrim3 == 0) {
          // repeated root case
          r1 = -b / (4 * a) - s4 / 2;
        }
        if (discrim4 > 0) {
          double sqrt2 = Math.sqrt(s5 + s6);
          r3 = -b / (4 * a) + s4 / 2 + sqrt2 / 2;
          r4 = -b / (4 * a) + s4 / 2 - sqrt2 / 2;
        } else if (discrim4 == 0) {
          r3 = -b / (4 * a) + s4 / 2;
        }
        if (discrim3 > 0 && discrim4 > 0)
          return new double[] { r1, r2, r3, r4 };
        else if (discrim3 > 0 && discrim4 == 0)
          return new double[] { r1, r2, r3 };
        else if (discrim3 > 0 && discrim4 < 0)
          return new double[] { r1, r2 };
        else if (discrim3 == 0 && discrim4 > 0)
          return new double[] { r1, r3, r4 };
        else if (discrim3 == 0 && discrim4 == 0)
          return new double[] { r1, r3 };
        else if (discrim3 == 0 && discrim4 < 0)
          return new double[] { r1 };
        else if (discrim3 < 0 && discrim4 > 0)
          return new double[] { r3, r4 };
        else if (discrim3 < 0 && discrim4 == 0)
          return new double[] { r3 };
        else if (discrim3 < 0 && discrim4 < 0)
          return new double[0];
      }
    }
    return new double[0];
  }

    // these time calculations are working correctly!! (estimated with counting seconds)
    private double[] calcTimeForBallToHitGoal(double pitchAngle, double xSpeed, double ySpeed, double xDist, double yDist, double zDist) {
      double a = -(4.9*4.9); 
      double b = 0; 
      double c = -9.8 * zDist + ((Math.tan(pitchAngle) * Math.tan(pitchAngle)) * ((xSpeed * xSpeed) + (ySpeed * ySpeed))); 
      double d = -2 * (Math.tan(pitchAngle) * Math.tan(pitchAngle)) * ((xSpeed * xDist) + (ySpeed * yDist)); 
      double e = -zDist + ((Math.tan(pitchAngle) * Math.tan(pitchAngle)) * ((xDist * xDist) + (yDist * yDist))); 

      // use function from stackoverflow to get roots
      double[] rawResults = solveRealQuarticRoots(a, b, c, d, e); 

      // filter roots
      java.util.List<Double> validRoots = new java.util.ArrayList<>();
      for (int i = 0; i < rawResults.length; i++) {
        double t = rawResults[i];
        Logger.recordOutput("Shooter/roots/" + i, t);
        // needs to exist and be positive (neg time isn't a thing)
        if (!Double.isNaN(t) && !Double.isInfinite(t) && t > 0) {
          validRoots.add(t);
        }
      }

      // convert valid values to array to return
      double[] filtered = new double[validRoots.size()];
      for (int i = 0; i < validRoots.size(); i++) {
        filtered[i] = validRoots.get(i);
      }
      return filtered;
    }

    private Translation3d getVectorFromRobotToTarget(Pose2d robotPose, Pose2d target) {
      Translation3d ballPositionVector = new Translation3d(robotPose.getX(), robotPose.getY(), Measurements.ShooterHeightFromGround);
      Translation3d targetGoalPositionVector = new Translation3d(target.getX(), target.getY(), getTargetHeight(target)); 
      Logger.recordOutput("Shooter/targetPose", targetGoalPositionVector); // this is correct/what I want

      Translation3d vectorFromRobotToTarget = targetGoalPositionVector.minus(ballPositionVector); // this is correct
      return vectorFromRobotToTarget; 
    }

    private double getDistBallFromTarget(Pose2d target) {
      Pose3d Target3D = new Pose3d(target.getX(), target.getY(), 0, new Rotation3d(0, 0,0)); 
      Pose3d landedBallPose = ball.getLandedPose3d(); 
      if (landedBallPose == null) {
        return Double.NaN; 
      }
      return landedBallPose.getTranslation().getDistance(Target3D.getTranslation()); 
    }

    public Pose3d[] makeAdvantageScopeLine(Translation3d vector, Pose2d robotPose, double heightOffset) {
      Pose3d[] final_array = new Pose3d[9]; 
      double robotX = robotPose.getX(); 
      double robotY = robotPose.getY(); 
      for (int i = 0; i < 9; i++) {
        Pose3d to_add = new Pose3d(vector.getX()/(i+1) + robotX, vector.getY()/(i+1) + robotY, heightOffset + vector.getZ()/(i+1), new Rotation3d(0,0,0)); 
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

      Translation3d robotVelocity3d = getRobotVector(robotPose); // m/s
      Translation3d vectorFromRobotToTarget = getVectorFromRobotToTarget(robotPose, target); // meters
      double[] times = calcTimeForBallToHitGoal(pitchAngle, robotVelocity3d.getX(), robotVelocity3d.getY(), vectorFromRobotToTarget.getX(), vectorFromRobotToTarget.getY(), vectorFromRobotToTarget.getZ());

      if (times.length == 0) {
        Logger.recordOutput("Shooter/calcdTimes/noValidTime", true);
        // set defaults if nothing found (could be impossible shot)
        lastCalculationPitchRadians = pitchAngle;
        lastCalculatedNeededSpeed = 0.0;
        lastCalculatedAngleInFieldFrame = 0.0;
        lastCalculationTurretAngleDegrees = 0.0;
        Logger.recordOutput("Shooter/ballToTarget", Double.NaN);
        return;
      }

      // log calculated times
      Logger.recordOutput("Shooter/calcdTimes/time1", times[0]);
      if (times.length > 1) {
        Logger.recordOutput("Shooter/calcdTimes/time2", times[1]);
        if (times.length > 2) {
          Logger.recordOutput("Shooter/calcdTimes/time3", times[2]);
          if (times.length > 3) {
            Logger.recordOutput("Shooter/calcdTimes/time4", times[3]);
          }
        }
      }

      // use first time for calculations (works in sim :) lol
      double time = times[0];
      Translation3d robotDisplacementTime = new Translation3d(robotVelocity3d.getX() * time, robotVelocity3d.getY() * time, robotVelocity3d.getZ() * time);

      Logger.recordOutput("Shooter/robotDisplacementTime", makeAdvantageScopeLine(robotDisplacementTime, robotPose, Measurements.ShooterHeightFromGround));

      Translation3d shooterSpeedVector = vectorFromRobotToTarget.minus(robotDisplacementTime); 

      Logger.recordOutput("Shooter/shooterSpeedVector", makeAdvantageScopeLine(shooterSpeedVector, robotPose, Measurements.ShooterHeightFromGround));
      Logger.recordOutput("Shooter/robotToTarget", makeAdvantageScopeLine(vectorFromRobotToTarget, robotPose, Measurements.ShooterHeightFromGround));

      Translation3d robotToTarget = getVectorFromRobotToTarget(robotPose, target); 
      Logger.recordOutput("Shooter/robotToTarget", makeAdvantageScopeLine(robotToTarget, robotPose, Measurements.ShooterHeightFromGround)); 
      Logger.recordOutput("Shooter/robotToTargetDistance", Math.sqrt((robotToTarget.getX() * robotToTarget.getX()) + (robotToTarget.getY() * robotToTarget.getY()) + (robotToTarget.getZ() * robotToTarget.getZ())));
      double dx = shooterSpeedVector.getX();
      double dy = shooterSpeedVector.getY();

      double horizontalDist = Math.sqrt(dx * dx + dy * dy);
      Logger.recordOutput("Shooter/horizontalDist", horizontalDist);

      double speed = horizontalDist / (time * Math.cos(pitchAngle));

      Logger.recordOutput("Shooter/speed", speed);

      double turretAngle = calculateTurretAngle(robotPose, shooterSpeedVector);
      Logger.recordOutput("Shooter/turretAngle", turretAngle);

      Logger.recordOutput("Shooter/origin", new Translation3d(0.0, 0.0, 0.0));

      lastCalculationPitchRadians = pitchAngle; // related to how the ball shooting is configured
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
      //double feedforwardVolts = feedforward.calculate(goalSpeed);
      double feedbackVolts = pid.calculate(getSpeed(), goalSpeed);
      double voltage =  feedbackVolts;
      Logger.recordOutput("Shooter/totalVoltage", voltage);

      Logger.recordOutput("Shooter/isInShootingMode", RobotContainer.getInstance().isInShootingMode()); 
      
      if (RobotContainer.getInstance().isRunningFlywheel) {
        if (Robot.isReal()) {
          //both should be positive
            flywheel1.setVoltage(voltage);
            flywheel2.setVoltage(voltage);
        } else {
            sim.setFlywheel1Voltage(voltage);
            sim.setFlywheel2Voltage(voltage);
            sim.update(0.02);
            ball.update();
        }
      } else {
        if (Robot.isReal()) {
          //both should be positive
            flywheel1.setVoltage(0);
            flywheel2.setVoltage(0);
        }  else {
          sim.setFlywheel1Voltage(0);
          sim.setFlywheel2Voltage(0);
          sim.update(0.02);
          ball.update();
      }
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

    public double getMotor1Voltage() {
      return flywheel1.getMotorVoltage().getValueAsDouble();
    }
    public double getMotor2Voltage() {
      return flywheel2.getMotorVoltage().getValueAsDouble();
    }
}
