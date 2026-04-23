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
    private TalonFX climbMotor;
    private ClimberSim sim = new ClimberSim();
    private double currentHeight = 0.0;
    public double sendingVolts = 0;
    public double maxHeight = 104.46;

      // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(ClimberConstants.kMaxVelocity.get(), ClimberConstants.kMaxAcceleration.get());
    private final ProfiledPIDController controller =
      new ProfiledPIDController(ClimberConstants.kP.get(), 0, ClimberConstants.kD.get(), constraints, 0.02);
    
    public static Climber getInstance() {
    if (instance == null)
      instance = new Climber();
    return instance;
  }

  public Climber() {
    climbMotor = new TalonFX(PortNumbers.Climb_Motor);
    climbMotor.setPosition(0);
  }

  public void setClimberChange(double posChange) {
    // controller.setGoal(controller.getGoal().position + posChange);
    // System.out.println(controller.getGoal().position + posChange);
    sendingVolts = sendingVolts + posChange;
  }


    
    public boolean getForwardLimit() {
        return false;
    }

  public void makeRotation() 
  {
    double calculate = controller.calculate(climbMotor.getPosition().getValueAsDouble(), controller.getGoal());
    climbMotor.setVoltage(calculate);
  }

    @Override 
    public void periodic() {
      //double calculate = controller.calculate(-climbMotor.getPosition().getValueAsDouble(), controller.getGoal());
      double calculate = sendingVolts;

      if (((climbMotor.getPosition().getValueAsDouble() > maxHeight) && (calculate > 0)) || (climbMotor.getPosition().getValueAsDouble() < -0.5)) {
        climbMotor.setVoltage(0);
      } else {
        climbMotor.setVoltage(calculate);
      }

      Logger.recordOutput("Climber/position", climbMotor.getPosition().getValueAsDouble());
      Logger.recordOutput("Climber/goal", sendingVolts);
      Logger.recordOutput("Climber/volts", climbMotor.getMotorVoltage().getValueAsDouble());
    }

    public void setVoltage(double volts) {
      System.out.println("SET VOLTAGE SET VOLTAGE");
      sendingVolts = volts;
    }

    public void setGoal(double goal) {
      controller.setGoal(goal);
    }

    public void setGoalToPosition() {
      System.out.println("SETTING GOAL");
      controller.setGoal(-climbMotor.getPosition().getValueAsDouble());
    }
   
}
