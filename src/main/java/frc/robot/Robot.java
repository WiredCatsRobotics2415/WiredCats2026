package frc.robot;

import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.constants.RuntimeConstants;
import frc.utils.LimelightHelpers; 

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    public Robot() {
        PowerDistribution pdh = new PowerDistribution();
        pdh.setSwitchableChannel(true);
        
        // DO NOT REMOVE THIS LINE
        // constructs the robot container, which constructs all subsystems
        RobotContainer.getInstance();

        setLimelightIMUMode(1); 
    }

    @Override
    public void simulationInit() {
        SimulatedArena.getInstance().placeGamePiecesOnField();
    }

    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
    }

    @Override
    public void robotPeriodic() {
        if (RuntimeConstants.TuningMode) {
            // TuneableNumber.periodic();
            // TuneableBoolean.periodic();
            // TorqueSafety.getInstance().periodic();
        }
        RobotContainer.getInstance().periodic();
        //run all commands that were added to the command scheduler
        CommandScheduler.getInstance().run();
        // if (RuntimeConstants.VisualizationEnabled) Visualizer.update();
    }

    @Override
    public void teleopInit() {
        setLimelightIMUMode(2); 

        RobotContainer.getInstance().teleopEnable();
    }

    @Override
    public void autonomousInit() {
        setLimelightIMUMode(2); 
        if (Robot.isSimulation()) SimulatedArena.getInstance().placeGamePiecesOnField();
        RobotContainer.getInstance().getAutonomousCommand().schedule();
    }

    @Override
    public void disabledInit() {
        setLimelightIMUMode(1); 
    }

    @Override
    public void disabledPeriodic() {
        setLimelightIMUMode(1);
    }

    @Override
    public void disabledExit() {
        // pass
    }

    private void setLimelightIMUMode(int mode) {
        if (Robot.isReal()) {
            LimelightHelpers.setLimelightNTDouble("limelight", "imumode_set", mode);
            LimelightHelpers.setLimelightNTDouble("limelight-2", "imumode_set", mode);
            LimelightHelpers.setLimelightNTDouble("limelight-3", "imumode_set", mode);
        }
    }
}