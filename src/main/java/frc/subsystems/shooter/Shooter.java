package frc.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import java.util.ArrayList;

import frc.constants.Measurements; 
import org.apache.commons.math3.stat.regression.SimpleRegression;

import org.littletonrobotics.junction.Logger;

public class Shooter {
    
    public Shooter() {
        
    }
    
    // get which speed file to use for angle regression
    private String getCSVForPose(Pose2d robotPose) {
        // for debugging
        Logger.recordOutput("Shooter/Hub Location", Measurements.HubLocation); 
        Logger.recordOutput("Shooter/Robot Pose", robotPose); 

        double distanceFromHub = robotPose.getTranslation().getDistance(Measurements.HubLocation.getTranslation());
        
        if (distanceFromHub <= Measurements.ShooterHubRegionOne) {
            return "angle_1_speed.csv"; // closer to hub
        } else {
            return "angle_2_speed.csv"; // further from hub
        }
    }

    public double getSpeedForDistance(Pose2d robotPose) {
        System.out.println("Shooter called"); 
        String filename = getCSVForPose(robotPose); 
        String filepath = "src/main/java/frc/constants/" + filename;

        ArrayList<Integer> speeds = new ArrayList<Integer>(); 
        ArrayList<Integer> distances = new ArrayList<Integer>(); 
        
        try (Scanner scanner = new Scanner(new File(filepath))) {
            scanner.useDelimiter(",|\\n");
            
            while (scanner.hasNext()) {
                if (scanner.hasNextInt()) {
                    int distance = scanner.nextInt();
                    if (scanner.hasNextInt()) {
                        int speed = scanner.nextInt();
                        speeds.add(speed); 
                        distances.add(distance); 
                    }
                } else {
                    scanner.next();
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

        double robotCurrentDistanceFromHub = robotPose.getTranslation().getDistance(Measurements.HubLocation.getTranslation()); 

        double speed = calculateSpeedForDistance.predict(robotCurrentDistanceFromHub);
        Logger.recordOutput("Shooter/Calculated Speed", speed); 
        Logger.recordOutput("Shooter/Distance from Hub", robotCurrentDistanceFromHub); 

        double angle; 

        if (filename.equals("angle_1_speed.csv")) {
            angle = Measurements.ShooterAngleLow; 
        } else {
            angle = Measurements.ShooterAngleHigh; 
        }

        Logger.recordOutput("Shooter/Angle", angle); 
        Logger.recordOutput("Shooter/CSV File Used", filename); 
        return speed; 
    }
}