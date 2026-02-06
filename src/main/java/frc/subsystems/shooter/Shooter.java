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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Measurements;
import frc.constants.Subsystems.ShooterConstants;
import frc.robot.Robot;
import frc.subsystems.drive.CommandSwerveDrivetrain;

// Import Subsystems Constants, TODO: currently all placeholders change once we have real values
public class Shooter extends SubsystemBase {

  private static Shooter instance = null;
  private static TalonFX flywheel1;
  private static TalonFX flywheel2;
  private static TalonFX indexerMotor;
  private static Integer FLYWHEEL_1_ID = 1;
  private static Integer FLYWHEEL_2_ID = 2;
  private static Integer INTAKE_MOTOR_ID = 3;
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
        // for debugging
        Logger.recordOutput("Shooter/Hub Location", Measurements.HubLocation); 
        Logger.recordOutput("Shooter/Robot Pose", robotPose); 

        if (getDegreesAngleForPose(robotPose) == 45) {
            return "low_angle_speed.csv"; // closer to hub
        } else {
            return "high_angle_speed.csv"; // further from hub
        }
    }

    public int getDegreesAngleForPose(Pose2d robotPose) {
      double distanceFromHub = robotPose.getTranslation().getDistance(Measurements.HubLocation.getTranslation());
      int angle; 

      if (distanceFromHub <= Measurements.ShooterHubRegionOne) {
          angle =  45; // closer to hub
      } else {
          angle = 15; // further from hub
      }

      return angle; 
    }

    public double getSpeedToHubForPose(Pose2d robotPose) {
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

        double robotCurrentDistanceFromHub = robotPose.getTranslation().getDistance(Measurements.HubLocation.getTranslation()) * 100; // returns meters * 100 = centimeters

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

    // uses given robot pose and current velocities to calculate actual needed speed
    // limitations: assumes robot speed remains constant, doesn't account for gravity/air resistance -> would require higher speed to overcome, this just gives exact
    public double getSpeedToHubForPoseAndVelocities(Pose2d robotPose) {
        double[] speed = getShooterSpeedAndAngleToHub(robotPose); 
        return speed[0]; 
    }

    public double[] getShooterSpeedAndAngleToHub(Pose2d robotPose) {
      double staticSpeed = getSpeedToHubForPose(robotPose); 
      System.out.println("IT'S WORKING");

      Pose2d hubPose = Measurements.HubLocation; 
      double angleToHub = hubPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians(); 

      double xNeeded = staticSpeed * Math.cos(angleToHub); 
      double yNeeded = staticSpeed * Math.sin(angleToHub); 

      CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
      ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getVelocity(), robotPose.getRotation());

      // Scale down the compensation to reduce over-correction
      double xCompensationFactor = 0.25; // Adjust this value between 0-1 to tune
      double yCompensationFactor = 0.25; // Adjust this value between 0-1 to tune
      double xVelocity = xNeeded - (fieldVel.vxMetersPerSecond * xCompensationFactor);
      double yVelocity = yNeeded - (fieldVel.vyMetersPerSecond * yCompensationFactor);

      double speedMagnitude = Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity);

      double angleInFieldFrame = Math.atan2(yVelocity, xVelocity);

      Logger.recordOutput("Shooter/Shooter Vel X", xVelocity);
      Logger.recordOutput("Shooter/Shooter Vel Y", yVelocity);
      Logger.recordOutput("Shooter/Speed with Velocity", speedMagnitude);
      Logger.recordOutput("Static angle", angleToHub);
      Logger.recordOutput("Angle In Field Frame", angleInFieldFrame);

      return new double[]{speedMagnitude, angleInFieldFrame}; 
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
