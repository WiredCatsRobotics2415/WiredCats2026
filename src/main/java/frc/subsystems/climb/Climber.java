package frc.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ClimberConstants;
import frc.constants.Subsystems.ShooterConstants;
import frc.robot.Robot;

public class Climber extends SubsystemBase {

    private static Climber instance = null;
    private AnalogInput wirePotentiometer;
    private TalonFX climbMotor = new TalonFX(50);
    private ClimberSim sim = new ClimberSim();
    private double currentHeight = 0.0;

      // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
    private final PIDController pid = new PIDController(ClimberConstants.kP, 0.0, 0.0);
    
    public static Climber getInstance() {
    if (instance == null)
      instance = new Climber();
    return instance;
  }

  private Climber() {
    wirePotentiometer = new AnalogInput(0);
    pid.setTolerance(0.05); 
  }

  public double getHeight() {
    return currentHeight / 3 * ClimberConstants.MaxHeight;
  }

    public Command SetVolt(double voltage) {
    return runOnce(
        () -> {
          pid.setSetpoint(voltage);
          System.out.println("Set climber setpoint to " + voltage);
        });
    }

    
    public boolean getForwardLimit() {
        return false;
    }

    public boolean getSideLimit() {
        return false;
    }

    @Override 
    public void periodic() {
      if (Robot.isReal()) {
        Logger.recordOutput("Climber/PotentiometerVoltage", wirePotentiometer.getVoltage());
        Logger.recordOutput("Climber/Voltage", climbMotor.getMotorVoltage().getValueAsDouble());
        currentHeight = wirePotentiometer.getVoltage();
        climbMotor.setVoltage(pid.calculate(wirePotentiometer.getVoltage(), pid.getSetpoint()) + ClimberConstants.kG);
      } else {
        sim.update(0.02);
        Logger.recordOutput("Climber/PotentiometerVoltage", sim.getPotentiometerVoltage());
        double voltage = (pid.calculate(sim.getPotentiometerVoltage(), pid.getSetpoint()) + ClimberConstants.kG);
        Logger.recordOutput("Climber/Voltage", voltage);
        currentHeight = sim.getPotentiometerVoltage();
        sim.setMotorVoltage(voltage);
      }
    }
}
