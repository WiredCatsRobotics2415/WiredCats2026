package frc.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ClimberConstants;
import frc.constants.Subsystems.ShooterConstants;

public class Climber extends SubsystemBase {

    private static Climber instance = null;
    private AnalogInput wirePotentiometer;
    private TalonFX climbMotor;


    private double goalPos = 0.0;

    private final PIDController pid = new PIDController(ClimberConstants.kP, 0.0, 0.0);
    
    public static Climber getInstance() {
    if (instance == null)
      instance = new Climber();
    return instance;
  }

  private Climber() {
    wirePotentiometer = new AnalogInput(0);
  }


  public double getGoalPos() {
    return goalPos;
  }

  public void setGoalPos(double goalPos) {
    this.goalPos = goalPos;
  }

    public Command Lock() {
    return runOnce(
        () -> {
          //drive left
          //move arm down
          //move arm up
        });
  }

    
    public boolean getForwardLimit() {
        return false;
    }

    public boolean getSideLimit() {
        return false;
    }

    public Command updatePos() {
        return runOnce(
                () -> {
                  climbMotor.setVoltage(pid.calculate(wirePotentiometer.getVoltage(), goalPos));
                });
    }

    @Override 
    public void periodic() {
        if (Math.abs(getGoalPos()) < ClimberConstants.GoalDeadband) {
            updatePos().schedule();
        }
    }
}
