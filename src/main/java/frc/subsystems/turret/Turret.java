package frc.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.subsystems.shooter.Shooter;
import frc.visualization.BallSim;
import frc.constants.Measurements; 

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
}