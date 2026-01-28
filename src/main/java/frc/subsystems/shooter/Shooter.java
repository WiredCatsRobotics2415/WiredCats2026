package frc.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;

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
    private String getCSVForPose(Pose3d robotPose) {
        double distanceFromHub = robotPose.toPose2d().getTranslation().getDistance(Measurements.HubLocation.getTranslation());
        
        if (distanceFromHub <= 2) {
            return "speed_1_angles.csv"; // closer to hub
        } else {
            return "speed_2_angles.csv"; // further from hub
        }
    }

    public double getAngleForDistance(Pose3d robotPose) {
        String filename = getCSVForPose(robotPose); 
        String filepath = "src/main/java/frc/constants/" + filename;

        ArrayList<Integer> angles = new ArrayList<Integer>(); 
        ArrayList<Integer> distances = new ArrayList<Integer>(); 
        
        try (Scanner scanner = new Scanner(new File(filepath))) {
            scanner.useDelimiter(",|\\n");
            
            while (scanner.hasNext()) {
                if (scanner.hasNextInt()) {
                    int distance = scanner.nextInt();
                    if (scanner.hasNextInt()) {
                        int angle = scanner.nextInt();
                        angles.add(angle); 
                        distances.add(distance); 
                    }
                } else {
                    scanner.next();
                }
            }
        } catch (IOException e) {
            System.err.println("Error reading CSV: " + e.getMessage());
        }

        SimpleRegression calculateAngleForDistance = new SimpleRegression(); 
        
        // add all datapoints to regression
        for (int i = 0; i < distances.size(); i++) {
            calculateAngleForDistance.addData(distances.get(i), angles.get(i));
        }

        double robotCurrentDistanceFromHub = robotPose.toPose2d().getTranslation().getDistance(Measurements.HubLocation.getTranslation()); 

        double angle = calculateAngleForDistance.predict(robotCurrentDistanceFromHub);
        Logger.recordOutput("Calculated Angle", angle); 
        Logger.recordOutput("Distance from Hub", robotCurrentDistanceFromHub); 

        double speed; 

        if (filename.equals("speed_1_angles.csv")) {
            speed = Measurements.ShooterRPMOne; 
        } else {
            speed = Measurements.ShooterRPMTwo; 
        }

        Logger.recordOutput("Shooter Speed", speed); 
        return angle; 
    }
}