// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems.shooter;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;
import frc.subsystems.shooter.ShooterSim;

import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Measurements;
import frc.constants.Subsystems.ShooterConstants;
import frc.robot.Robot;
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

  //WE ARE USING SIMPLE FEEDFORWARD AND PID BECAUSE WE ARE SETTING FLYWHEEL VELOCITIES AND DON'T NEED POSITION ACCURACY
  private final PIDController pid = new PIDController(ShooterConstants.kP, 0.0, 0.0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

  private static ShooterSim sim = new ShooterSim();

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
    //starting it at 50%
    indexerMotor.set(0.5);
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

  //Charlotte positioning/regression
  // get which speed file to use for angle regression
    private String getCSVForPose(Pose2d robotPose) {
        if (getDegreesAngleForPose(robotPose) == 45) {
            return "low_angle_speed.csv"; // closer to hub
        } else {
            return "high_angle_speed.csv"; // further from hub
        }
    }

    public int getDegreesAngleForPose(Pose2d robotPose) {
      int angle; 

      if (Measurements.ShootIntoHubRegion.contains(robotPose.getTranslation())) {
          angle = Measurements.ShooterAngleLow; // closer to hub (inside region)
      } else {
          angle = Measurements.ShooterAngleHigh; // further from hub (outside region)
      }

      return angle; 
    }

    public double getSpeedToTargetForPose(Pose2d robotPose, Pose2d target) {
        String filename = getCSVForPose(robotPose); 
        String filepath = "src/main/java/frc/constants/" + filename;

        ArrayList<Double> speeds = new ArrayList<Double>(); 
        ArrayList<Integer> distances = new ArrayList<Integer>();
        
        try (Scanner scanner = new Scanner(new File(filepath))) {
            // skip header line
            if (scanner.hasNextLine()) {
                scanner.nextLine();
            }
            
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String[] parts = line.split(",");
                if (parts.length == 2) {
                    int distance = Integer.parseInt(parts[0]);
                    double speed = Double.parseDouble(parts[1]);
                    speeds.add(speed); 
                    distances.add(distance);
                }
            }
        } catch (IOException e) {
            System.err.println("Error reading CSV: " + e.getMessage());
        }

        SimpleRegression calculateSpeedForDistance = new SimpleRegression(); 
        
        // add all datapoints to regression
        for (int i = 0; i < distances.size(); i++) {
            calculateSpeedForDistance.addData(distances.get(i), speeds.get(i));
        }

        double robotCurrentDistanceFromHub = robotPose.getTranslation().getDistance(target.getTranslation()) * 100; // returns meters * 100 = centimeters

        double motorSpeed = calculateSpeedForDistance.predict(robotCurrentDistanceFromHub);
        // clamp speed to 20 + (keeps positive)
        motorSpeed = Math.max(motorSpeed, 20); 

        double speed = motorSpeed * 0.37;

        Logger.recordOutput("Shooter/Calculated Speed", speed); 
        Logger.recordOutput("Shooter/Distance from Hub", robotCurrentDistanceFromHub); 

        double angle; 

        if (filename.equals("low_angle_speed.csv")) {
            angle = Measurements.ShooterAngleLow; 
        } else {
            angle = Measurements.ShooterAngleHigh; 
        }

        Logger.recordOutput("Shooter/Angle", angle); 
        Logger.recordOutput("Shooter/CSV File Used", filename); 
        
        return speed; 
    }


    public double[] getShooterSpeedAndAngleToTarget(Pose2d robotPose, Pose2d target, int shooterAngle) {
      double staticSpeed = getSpeedToTargetForPose(robotPose, target); 

      double angleToTarget = target.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians(); 

      double xNeeded = staticSpeed * Math.cos(angleToTarget); 
      double yNeeded = staticSpeed * Math.sin(angleToTarget); 

      CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
      ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getVelocity(), robotPose.getRotation());

      // Scale down the compensation to reduce over-correction
      // CHARLOTTE NOTE: I think we may need a more agressive x compensation on 45 degrees
      double xCompensationFactor = 0.25; // Adjust this value between 0-1 to tune
      double yCompensationFactor = 0.25; // Adjust this value between 0-1 to tune
      double xVelocity = xNeeded - (fieldVel.vxMetersPerSecond * xCompensationFactor);
      double yVelocity = yNeeded - (fieldVel.vyMetersPerSecond * yCompensationFactor);

      double speedMagnitude = Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity);

      double angleInFieldFrame = Math.atan2(yVelocity, xVelocity);

      Logger.recordOutput("Shooter/Shooter Vel X", xVelocity);
      Logger.recordOutput("Shooter/Shooter Vel Y", yVelocity);
      Logger.recordOutput("Shooter/Speed with Velocity", speedMagnitude);
      Logger.recordOutput("Static angle to target", angleToTarget);
      Logger.recordOutput("Angle In Field Frame", angleInFieldFrame);

      return new double[]{speedMagnitude, angleInFieldFrame}; 
    }

    public void setTurretAndShooterForPose(Pose2d robotPose) {
        int shooterAngleDegrees = getDegreesAngleForPose(robotPose);
        
        // Determine target
        Pose2d target = getTargetForAngle(robotPose, shooterAngleDegrees);
        
        // Get shooting parameters
        double[] speedAndAngle = getShooterSpeedAndAngleToTarget(robotPose, target, shooterAngleDegrees);
        double speedMagnitude = speedAndAngle[0];
        double angleInFieldFrame = speedAndAngle[1];
        
        double turretAngleRadians = angleInFieldFrame - robotPose.getRotation().getRadians() + Math.toRadians(Measurements.TurretAngleOffset);
        
        double pitchRadians = Math.toRadians(shooterAngleDegrees - 90); 
        
        logShootingData(robotPose, target, speedMagnitude, angleInFieldFrame, 
                        turretAngleRadians, shooterAngleDegrees, pitchRadians);

        if (Robot.isReal()) {
            //TODO: set turret angle
        } else {
            ball.throwBall(
            new Pose3d(robotPose.getX(), robotPose.getY(), 0.457, new Rotation3d()),
            new Rotation3d(0, pitchRadians, angleInFieldFrame),
            speedMagnitude
        );
        }
        setGoalSpeed(speedMagnitude);
    }

    private Pose2d getTargetForAngle(Pose2d robotPose, int shooterAngle) {
        if (shooterAngle != 45) {
            return Measurements.HubLocation;
        }
        // low angle, selects closest target for left or right
        return (robotPose.getTranslation().getDistance(Measurements.LeftMidAllianceRegion.getTranslation()) < 
                robotPose.getTranslation().getDistance(Measurements.RightMidAllianceRegion.getTranslation())) 
                ? Measurements.LeftMidAllianceRegion : Measurements.RightMidAllianceRegion;
    }

    private void logShootingData(Pose2d robotPose, Pose2d target, double speed, double angleInFieldFrame, double turretAngle, int shooterAngle, double pitchRadians) {
        Logger.recordOutput("Shooter/Target", target);
        Logger.recordOutput("Shooter/Shooter Speed", speed);
        Logger.recordOutput("Shooter/Angle (deg)", shooterAngle);
        Logger.recordOutput("Shooter/Pitch (rad)", pitchRadians);

        Logger.recordOutput("Turret/Robot Pose", robotPose);
        Logger.recordOutput("Turret/Angle In Field Frame (rad)", angleInFieldFrame);
        Logger.recordOutput("Turret/Turret Angle (rad)", turretAngle);
        Logger.recordOutput("Turret/Turret Angle (deg)", Math.toDegrees(turretAngle));
        Logger.recordOutput("Turret/Robot Rotation (deg)", robotPose.getRotation().getDegrees());

        Logger.recordOutput("Turret/Robot Pose", robotPose);
        Logger.recordOutput("Shooter/Hub Location", Measurements.HubLocation); 
        Logger.recordOutput("Shooter/Robot Pose", robotPose); 
        Logger.recordOutput("Shooter/ShootIntoHubRegion", Measurements.ShootIntoHubRegion); 
        Logger.recordOutput("Shooter/LeftMidAllianceRegion", Measurements.LeftMidAllianceRegion); 
        Logger.recordOutput("Shooter/RightMidAllianceRegion", Measurements.RightMidAllianceRegion); 
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

      if (Robot.isReal()) {
            flywheel1.setVoltage(voltage);
            flywheel2.setVoltage(voltage);
    } else {
        sim.setFlywheel1Voltage(voltage);
        sim.setFlywheel2Voltage(voltage);
        sim.update(0.02);
    }
  }

}
