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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ClimberConstants;
import frc.constants.Subsystems.IntakeConstants;
import frc.constants.Subsystems.ShooterConstants;
import frc.constants.Subsystems.PortNumbers;
import frc.robot.Robot;

public class Climber extends SubsystemBase {

  private static Climber instance = null;
  private TalonFX climbMotor = new TalonFX(PortNumbers.Climb_Motor);
  private ClimberSim sim = new ClimberSim();
  private DigitalInput limitSwitchLeft = new DigitalInput(PortNumbers.LimitSwitchLeft_ID);
  private DigitalInput limitSwitchRight = new DigitalInput(PortNumbers.LimitSwitchRight_ID);
  private boolean leftRight = false; //False = left, Right = true. If left (meaning left hook is all the way down), bring right hook up and vice versa
  private boolean climberRunning = false;


    // Create a PID controller whose setpoint's change is subject to maximum
// velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints constraints =
    new TrapezoidProfile.Constraints(ClimberConstants.kMaxVelocity, ClimberConstants.kMaxAcceleration);
  private final ProfiledPIDController controller =
    new ProfiledPIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, constraints, 0.02);
    
    public static Climber getInstance() {
    if (instance == null)
      instance = new Climber();
    return instance;
  }

  private Climber() {
    controller.setTolerance(0.05); 
  }

  public double getHeight() {
    return climbMotor.getPosition().getValueAsDouble(); //not accurate, in rotations
  }

  public void setClimberState(boolean turnOnOrNot)
  {
    climberRunning = turnOnOrNot;
    if(climberRunning) {makeRotation();}
  }

  public void makeRotation() 
  {
    double calculate = controller.calculate(climbMotor.getPosition().getValueAsDouble(), controller.getGoal());
    climbMotor.setVoltage(calculate);
  }

    @Override 
    public void periodic() {
      if (Robot.isReal() && climberRunning) {
        makeRotation();

        if(limitSwitchLeft.get())
        {
          //leftRight = false;
          controller.setGoal(ClimberConstants.amountToMove); //May need to switch these values if going wrong way
        }
        if(limitSwitchRight.get())
        {
          //leftRight = true;
          controller.setGoal(-ClimberConstants.amountToMove);
        }
      }
      else if(!climberRunning)
      {
        climbMotor.setVoltage(0);
      }
    }
   
}
