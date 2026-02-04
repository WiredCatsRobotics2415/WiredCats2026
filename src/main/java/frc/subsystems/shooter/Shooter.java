// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems.shooter;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Measurements;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.turret.Turret;

// Import Subsystems Constants, TODO: currently all placeholders change once we have real values
public class Shooter extends SubsystemBase {

  private static Shooter instance = null;
  private static TalonFX flywheel1;
  private static TalonFX flywheel2;
  private static TalonFX intakeMotor;
  private static Integer FLYWHEEL_1_ID = 1;
  private static Integer FLYWHEEL_2_ID = 2;
  private static Integer INTAKE_MOTOR_ID = 3;

  //feedforward
  private static Slot0Configs slot0Configs;
  private static VelocityVoltage request;

  // Intake Functions Below
  private Shooter() {
    // Constructor, idk what to put here rn
    flywheel1 = new TalonFX(FLYWHEEL_1_ID);
    flywheel2 = new TalonFX(FLYWHEEL_2_ID);

    // Configure feedforward for flywheels
    slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0.5; // An error of 1 rps increases output by 0.5 V each second
    slot0Configs.kD = 0.01; // An acceleration of 1 rps/s results in 0.01 V output

    flywheel1.getConfigurator().apply(slot0Configs);
    flywheel2.getConfigurator().apply(slot0Configs);

    request = new VelocityVoltage(0).withSlot(0);

    intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
  }

  public static Shooter getInstance() {
    if (instance == null)
      instance = new Shooter();
    return instance;
  }

  public void startShooting() // Starts Intake
  {
    //starting it at 50%
    flywheel1.set(0.5);
  }

  public void stopShooting() // Stops Intake
  {
    intakeMotor.set(0.0);
  }

  // Flywheel Functions Below
  public float getSpeed() // Returns average speed of both flywheels
  {
    return (float) ((flywheel1.getVelocity().getValueAsDouble() + flywheel2.getVelocity().getValueAsDouble()) / 2);
  }

  public void speedUp(float speed) // Sets Flywheel speed, ask if Override with set SpeedUp paramater is needed
  {
    flywheel1.setControl(request.withVelocity(speed).withFeedForward(0.5));
    flywheel1.setControl(request.withVelocity(speed).withFeedForward(0.5));
  }

  public void speedDown() // Stops Flywheel
  {
    flywheel1.set(0.0);
    flywheel2.set(0.0);
  }

  //Charlotte positioning/regression
  // get which speed file to use for angle regression
    private String getCSVForPose(Pose2d robotPose) {
        // for debugging
        Logger.recordOutput("Shooter/Hub Location", Measurements.HubLocation); 
        Logger.recordOutput("Shooter/Robot Pose", robotPose); 

        double distanceFromHub = robotPose.getTranslation().getDistance(Measurements.HubLocation.getTranslation());

        if (distanceFromHub <= Measurements.ShooterHubRegionOne) {
            return "low_angle_speed.csv"; // closer to hub
        } else {
            return "high_angle_speed.csv"; // further from hub
        }
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

      Pose2d hubPose = Measurements.HubLocation; 
      double angleToHub = hubPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians(); 

      double xNeeded = staticSpeed * Math.cos(angleToHub); 
      double yNeeded = staticSpeed * Math.sin(angleToHub); 

      CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
      ChassisSpeeds currVelocities = drive.getVelocity();
      ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getVelocity(), robotPose.getRotation());
      
      double xVelocity = xNeeded - fieldVel.vxMetersPerSecond;
      double yVelocity = yNeeded - fieldVel.vyMetersPerSecond;

      double speedMagnitude = Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity);

      double angleInFieldFrame = Math.atan2(yVelocity, xVelocity);

      Logger.recordOutput("Shooter/Robot Relative Vel X", currVelocities.vxMetersPerSecond);
      Logger.recordOutput("Shooter/Robot Relative Vel Y", currVelocities.vyMetersPerSecond);
      Logger.recordOutput("Shooter/Field Relative Vel X", fieldVel.vxMetersPerSecond);
      Logger.recordOutput("Shooter/Field Relative Vel Y", fieldVel.vyMetersPerSecond);
      Logger.recordOutput("Shooter/Shooter Vel X", xVelocity);
      Logger.recordOutput("Shooter/Shooter Vel Y", yVelocity);
      Logger.recordOutput("Shooter/Speed w/ Velocity", speedMagnitude);
      Logger.recordOutput("Shooter/Angle In Field Frame", angleInFieldFrame);

      return new double[]{speedMagnitude, angleInFieldFrame}; 
    }

}
