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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ClimberConstants;
import frc.constants.Subsystems.ShooterConstants;
import frc.constants.Subsystems.PortNumbers;
import frc.robot.Robot;

public class Climber extends SubsystemBase {

    private static Climber instance = null;
    private TalonFX climbMotor = new TalonFX(PortNumbers.Climb_Motor);
    private ClimberSim sim = new ClimberSim();
    private double currentHeight = 0.0;
    private DigitalInput limitSwitchLeft = new DigitalInput(PortNumbers.LimitSwitchLeft_ID);
    private DigitalInput limitSwitchRight = new DigitalInput(PortNumbers.LimitSwitchRight_ID);
    private boolean leftRight = false; //False = left, Right = true. If left (meaning left hook is all the way down), bring right hook up and vice versa
    private boolean climberRunning = false;


      // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
    private final PIDController pid = new PIDController(ClimberConstants.kP, 0.0, 0.0);
    
    public static Climber getInstance() {
    if (instance == null)
      instance = new Climber();
    return instance;
  }

  private Climber() {
    pid.setTolerance(0.05); 
  }

  public double getHeight() {
    return currentHeight / 3 * ClimberConstants.MaxHeight;
  }

  public void SetClimberState(boolean turnOnOrNot)
  {
    climberRunning = turnOnOrNot;
    if(climberRunning) {SetVoltage();}
  }

    public void SetVoltage() 
    {
      //TODO: Not sure what voltage goes which direction, change later
      climbMotor.setVoltage(leftRight ? /*Move Left Hook Up*/ClimberConstants.ClimberVoltage : /*Move RIght Hook Up*/-ClimberConstants.ClimberVoltage);
    }

    @Override 
    public void periodic() {
      if (Robot.isReal() && climberRunning) {
        if(limitSwitchLeft.get())
        {
          leftRight = false;
          SetVoltage();
        }
        if(limitSwitchRight.get())
        {
          leftRight = true;
          SetVoltage();
        }
      }
    }
}
